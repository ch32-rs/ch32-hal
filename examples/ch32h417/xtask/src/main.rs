//! Build, flash and merge driver for the CH32H417 dual-core examples.
//!
//! H417 is one chip with two cores that cannot share a Cargo target: the V3F
//! (control core, hart 0) is a 32-bit `imafc` core, while the V5F (hart 1) is
//! `imafbc` with its own memory map part-way into the same flash. They are
//! therefore two crates under this directory, and this xtask is the entry
//! point that knows how to build them, flash them in the right order, and
//! merge them into a single image.
//!
//! ```text
//! cargo xtask build [--example NAME] [--core v3f|v5f]
//! cargo xtask flash --example NAME [--core v3f|v5f] [--no-build]
//! cargo xtask run   --example NAME [--core v3f|v5f] [--no-build]
//! cargo xtask merge --example NAME [--out DIR]
//! ```
//!
//! # The two kinds of example
//!
//! Reset only starts the V3F: the V5F sits until the V3F writes its wake
//! register, so a V5F-only image has no entry point and cannot run by itself.
//! Examples therefore come in exactly two shapes, and the xtask derives which
//! one from the files that exist:
//!
//! * **v3f-only** — `v3f/src/bin/NAME.rs` alone.
//! * **dual-core** — `v3f/src/bin/NAME.rs` *and* `v5f/src/bin/NAME.rs`; the
//!   V3F half brings the chip up and wakes hart 1.
//!
//! A `v5f/src/bin/NAME.rs` without its V3F half is rejected.
//!
//! Every `cargo`/`wlink` invocation runs with the working directory set to the
//! core's crate: that is what makes the crate's `.cargo/config.toml` (custom
//! target JSON and `build-std`) apply. Running `cargo --manifest-path v3f/…`
//! from here would silently fall back to the host target.

use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::{exit, Command};
use std::thread;
use std::time::{Duration, Instant};

/// Bytes an erased flash byte reads as, used to pad the merged image.
const ERASED: u8 = 0xFF;

/// Where the chip's flash starts in the address space; `Core::flash_base` is
/// relative to it.
const CHIP_FLASH_BASE: u64 = 0x0800_0000;

/// How many times a flash is retried before giving up.
const VERIFY_ATTEMPTS: u32 = 3;

/// Deadline for a write, generous enough that a healthy one is never cut short.
const FLASH_TIMEOUT: Duration = Duration::from_secs(60);

/// How long to let the link settle after a mode switch or chip reset.
const RECOVER_SETTLE: Duration = Duration::from_secs(2);

/// Deadline for a request that only moves registers or a few bytes.
const QUICK_TIMEOUT: Duration = Duration::from_secs(30);

/// One core of the chip.
///
/// `flash_base` / `flash_limit` mirror that core's `memory.x` (the V3F owns the
/// first 64K and the V5F image starts at 0x00010000, the same split the WCH
/// CSDK uses in `Ld/V3F/Link_v3f.ld` and `Ld/V5F/Link_v5f.ld`).
struct Core {
    /// Directory name below this one, and the value of `--core`.
    name: &'static str,
    /// Rust target of the custom JSON in that directory.
    target: &'static str,
    /// Offset of this core's image inside the chip's flash.
    flash_base: u64,
    /// Flash bytes the image may occupy.
    flash_limit: u64,
}

/// In flash order: the V3F image is written first with `--no-run`, then the V5F
/// image resets and runs the chip, by which time both images are in place.
const CORES: [Core; 2] = [
    Core {
        name: "v3f",
        target: "riscv32imafc-unknown-none-elf",
        flash_base: 0x0000_0000,
        flash_limit: 64 * 1024,
    },
    Core {
        name: "v5f",
        target: "riscv32imafbc-unknown-none-elf",
        flash_base: 0x0001_0000,
        flash_limit: 960 * 1024 - 64 * 1024,
    },
];

/// The core that reset starts, and the only one an example may be limited to.
const BOOT_CORE: &str = "v3f";

const USAGE: &str = "\
usage: cargo xtask <command> [options]

commands:
  build [--example NAME] [--core v3f|v5f]   build one example (or everything)
  flash --example NAME [--core v3f|v5f] [--no-build]
                                            build, then flash and reset
  run   --example NAME [--core v3f|v5f] [--no-build]
                                            like flash, plus the SDI console
  merge --example NAME [--out DIR]          merge the images into .bin + .hex

Examples are either v3f-only or dual-core; see the module docs.";

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let Some((command, rest)) = args.split_first() else {
        fail(USAGE);
    };

    let opts = Options::parse(rest);
    match command.as_str() {
        "build" => build(&opts),
        "flash" => flash(&opts, false),
        "run" => flash(&opts, true),
        "merge" => merge(&opts),
        "help" | "--help" | "-h" => println!("{USAGE}"),
        other => fail(&format!("unknown command `{other}`\n\n{USAGE}")),
    }
}

/// Parsed command line.
struct Options {
    example: Option<String>,
    core: Option<String>,
    no_build: bool,
    out: Option<PathBuf>,
}

impl Options {
    fn parse(args: &[String]) -> Options {
        let mut opts = Options {
            example: None,
            core: None,
            no_build: false,
            out: None,
        };
        let mut args = args.iter();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--example" => opts.example = args.next().cloned(),
                "--core" => opts.core = args.next().cloned(),
                "--out" => opts.out = args.next().map(PathBuf::from),
                "--no-build" => opts.no_build = true,
                other => fail(&format!("unknown option `{other}`\n\n{USAGE}")),
            }
        }
        if let Some(core) = &opts.core {
            if !CORES.iter().any(|c| c.name == core) {
                fail(&format!("unknown core `{core}` (expected v3f or v5f)"));
            }
        }
        opts
    }

    /// Cores to act on: everything, or `--core`, intersected with the cores
    /// that actually carry the example.
    ///
    /// Rejects a V5F-only example — hart 1 has no entry point of its own.
    fn select(&self) -> Vec<&'static Core> {
        let carrying: Vec<&'static Core> = match &self.example {
            Some(example) => CORES
                .iter()
                .filter(|core| example_source(core, example).exists())
                .collect(),
            None => CORES.iter().collect(),
        };

        if carrying.is_empty() {
            let example = self.example.as_deref().unwrap_or_default();
            fail(&format!("no core carries the `{example}` example"));
        }
        if !carrying.iter().any(|core| core.name == BOOT_CORE) {
            let example = self.example.as_deref().unwrap_or_default();
            fail(&format!(
                "`{example}` only exists for the V5F, which cannot start on its own:\n\
                 \x20     reset runs the V3F, so every example needs a v3f/src/bin/{example}.rs\n\
                 \x20     half that wakes hart 1"
            ));
        }

        let selected: Vec<&'static Core> = carrying
            .into_iter()
            .filter(|core| self.core.as_deref().is_none_or(|want| want == core.name))
            .collect();
        if selected.is_empty() {
            let example = self.example.as_deref().unwrap_or_default();
            fail(&format!(
                "the `{example}` example has no source for the requested core"
            ));
        }
        selected
    }
}

/// Directory holding `v3f/`, `v5f/`, `xtask/` and `out/`.
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("xtask lives one level below the H417 examples directory")
        .to_path_buf()
}

/// Source file of `example` on `core`, if that core carries it.
fn example_source(core: &Core, example: &str) -> PathBuf {
    root()
        .join(core.name)
        .join("src/bin")
        .join(format!("{example}.rs"))
}

/// Built (release) ELF of `example` on `core`.
fn example_elf(core: &Core, example: &str) -> PathBuf {
    root()
        .join(core.name)
        .join("target")
        .join(core.target)
        .join("release")
        .join(example)
}

/// Runs a program from `dir`, failing the xtask if it does.
fn run(program: &str, args: &[&str], dir: &Path) {
    println!("  {} {}", program, args.join(" "));
    let status = Command::new(program)
        .args(args)
        .current_dir(dir)
        .status()
        .unwrap_or_else(|e| fail(&format!("failed to run `{program}`: {e}")));
    if !status.success() {
        fail(&format!("`{program}` failed with {status}"));
    }
}

/// Runs `program`, killing it if it outlives `timeout`. Returns success.
///
/// Writes need this: when an earlier program was interrupted the chip's flash
/// controller stays mid-operation, and the next `wlink flash` then waits forever
/// for a ready bit that never arrives. Without a deadline the whole xtask hangs
/// there, so the caller can reset the chip and retry instead.
fn run_timed(program: &str, args: &[&str], dir: &Path, timeout: Duration) -> bool {
    let mut child = match Command::new(program)
        .args(args)
        .current_dir(dir)
        .spawn()
    {
        Ok(child) => child,
        Err(e) => {
            println!("  failed to run `{program}`: {e}");
            return false;
        }
    };
    let deadline = Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(status)) => return status.success(),
            Ok(None) => {}
            Err(e) => {
                println!("  failed to wait for `{program}`: {e}");
                return false;
            }
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            println!("  `{program}` did not finish within {}s", timeout.as_secs());
            return false;
        }
        thread::sleep(Duration::from_millis(100));
    }
}

/// Escalates between write attempts.
///
/// The one recovery observed reproducibly is simply repeating the write: a
/// request that follows a killed one often hangs once and then works. The
/// stronger steps — reset the chip, then re-cycle the link's protocol mode
/// (RV -> DAP -> RV) — are best-effort: they are cheap and have cleared the few
/// persistent cases, but the evidence is anecdotal (one such "recovery" was
/// really the board being restarted by hand), so they only run after a plain
/// retry has already failed.
fn recover(attempt: u32) {
    if attempt == 1 {
        println!("  retrying ...");
        return;
    }
    println!("  resetting the chip and re-cycling the link mode before the retry ...");
    run_timed("wlink", &["reset", "halt"], &root(), QUICK_TIMEOUT);
    thread::sleep(RECOVER_SETTLE);
    run_timed("wlink", &["mode-switch", "--dap"], &root(), QUICK_TIMEOUT);
    thread::sleep(RECOVER_SETTLE);
    for _ in 0..2 {
        if run_timed("wlink", &["mode-switch", "--rv"], &root(), QUICK_TIMEOUT) {
            break;
        }
        thread::sleep(RECOVER_SETTLE);
    }
    // The link needs a moment in RV mode before it will program again.
    thread::sleep(RECOVER_SETTLE);
}

fn build(opts: &Options) {
    let cores = opts.select();
    for core in cores {
        let dir = root().join(core.name);
        let mut args = vec!["build", "--release"];
        if let Some(example) = &opts.example {
            args.extend(["--bin", example]);
        }
        println!("building {} ...", core.name);
        run("cargo", &args, &dir);
    }
    if opts.example.is_some() {
        println!("built: {}", describe(&opts.select()));
    }
}

/// `flash` (watch = false) and `run` (watch = true) share this.
///
/// The non-final cores are written with `--no-run` and read back, so nothing
/// starts half-programmed. The final write resets and runs the chip and carries
/// the SDI flags — and nothing may follow it, because *every* other wlink
/// request (`dump`, `regs`, `status`) pauses the cores: a read-back check there
/// would measure the paused state instead of the running one.
fn flash(opts: &Options, watch: bool) {
    let Some(example) = &opts.example else {
        fail(&format!(
            "`{}` needs --example\n\n{USAGE}",
            if watch { "run" } else { "flash" }
        ));
    };
    let cores = opts.select();
    if !opts.no_build {
        build(opts);
    }

    for (i, core) in cores.iter().enumerate() {
        let elf = example_elf(core, example);
        let last = i + 1 == cores.len();
        println!("flashing {} ...", core.name);

        if last {
            // The final write resets and runs the chip. `run` adds the SDI pair
            // (`--enable-sdi-print --watch-serial`: the console needs the target
            // running, which is also what keeps hart 1 alive); `flash` stays
            // plain, because enabling SDI print without watching leaves the
            // debug module attached and the second core paused.
            let mut args = vec!["flash"];
            if watch {
                if is_dual_core(example) {
                    println!(
                        "note: watching keeps wlink attached, and any wlink request pauses the\n\
                         \x20     cores — while the console is open the V5F half cannot run.\n\
                         \x20     Use `cargo xtask flash` and read the mailbox afterwards."
                    );
                }
                args.extend(["--enable-sdi-print", "--watch-serial"]);
            }
            args.push(elf.to_str().expect("utf-8 path"));
            run("wlink", &args, &root());
        } else {
            let args = [
                "flash",
                "--no-run",
                elf.to_str().expect("utf-8 path"),
            ];
            program_verified(core, example, &elf, &args, true);
        }
    }
}

/// Whether `example` has a V5F half, i.e. it is a dual-core example.
fn is_dual_core(example: &str) -> bool {
    CORES
        .iter()
        .filter(|core| core.name != BOOT_CORE)
        .any(|core| example_source(core, example).exists())
}

/// Runs `wlink` with `args`, then reads the region back and compares it with the
/// image that was built (`check`). Retries the whole command on mismatch.
///
/// `wlink flash` has been observed to return success while only part of the
/// image — or none of it — reached the flash: the log stops after
/// `Read protected: false` and never prints `Flash done`. Without this check
/// that looks exactly like a firmware bug in the code under test.
fn program_verified(core: &Core, example: &str, elf: &Path, args: &[&str], check: bool) {
    let expected = flatten(elf, core);
    for attempt in 1..=VERIFY_ATTEMPTS {
        let wrote = run_timed("wlink", args, &root(), FLASH_TIMEOUT);
        if wrote && !check {
            return;
        }
        if !wrote {
            recover(attempt);
            continue;
        }
        match read_flash(core, expected.len()) {
            // `wlink dump` rounds its length up to a word, so compare the
            // image-sized prefix rather than the vectors themselves.
            Some(actual) if actual.len() >= expected.len() && actual[..expected.len()] == expected => {
                println!("  {} verified ({} bytes)", core.name, expected.len());
                return;
            }
            Some(actual) => println!(
                "  {}: read-back differs ({} of {} bytes match) on attempt {attempt}",
                core.name,
                actual
                    .iter()
                    .zip(&expected)
                    .take_while(|(a, b)| a == b)
                    .count(),
                expected.len()
            ),
            None => println!("  {}: read-back failed on attempt {attempt}", core.name),
        }
        recover(attempt);
    }
    fail(&format!(
        "{}: `{example}` is not in flash after {VERIFY_ATTEMPTS} attempts — \
         the probe returned success but did not program the image",
        core.name
    ));
}

/// Reads `len` bytes of `core`'s image back out of flash, or `None` on failure.
fn read_flash(core: &Core, len: usize) -> Option<Vec<u8>> {
    let tmp = env::temp_dir().join(format!("h417-verify-{}-{}.bin", core.name, std::process::id()));
    let address = format!("{:#010x}", CHIP_FLASH_BASE + core.flash_base);
    let length = len.to_string();
    let ok = run_timed(
        "wlink",
        &["dump", &address, &length, "-o", tmp.to_str()?],
        &root(),
        QUICK_TIMEOUT,
    );
    let bytes = ok.then(|| fs::read(&tmp).ok()).flatten();
    let _ = fs::remove_file(&tmp);
    bytes
}

/// A contiguous payload of the merged image.
struct Segment {
    core: &'static str,
    base: u64,
    bytes: Vec<u8>,
}

fn merge(opts: &Options) {
    let Some(example) = &opts.example else {
        fail(&format!("`merge` needs --example\n\n{USAGE}"));
    };

    let segments: Vec<Segment> = opts
        .select()
        .into_iter()
        .map(|core| Segment {
            core: core.name,
            base: core.flash_base,
            bytes: flatten(&example_elf(core, example), core),
        })
        .collect();

    // Assemble the full image: erased flash, with each core's payload at its
    // own offset.
    let end = segments
        .iter()
        .map(|s| s.base + s.bytes.len() as u64)
        .max()
        .expect("at least one segment");
    let mut image = vec![ERASED; end as usize];
    for segment in &segments {
        let at = segment.base as usize;
        image[at..at + segment.bytes.len()].copy_from_slice(&segment.bytes);
    }

    let out_dir = opts.out.clone().unwrap_or_else(|| root().join("out"));
    fs::create_dir_all(&out_dir)
        .unwrap_or_else(|e| fail(&format!("cannot create {}: {e}", out_dir.display())));
    let bin = out_dir.join(format!("{example}.bin"));
    let hex = out_dir.join(format!("{example}.hex"));
    fs::write(&bin, &image).unwrap_or_else(|e| fail(&format!("cannot write {}: {e}", bin.display())));
    fs::write(&hex, intel_hex(&segments))
        .unwrap_or_else(|e| fail(&format!("cannot write {}: {e}", hex.display())));

    for segment in &segments {
        println!(
            "  {:<4} {:#010x}  {:>7} bytes",
            segment.core,
            segment.base,
            segment.bytes.len()
        );
    }
    println!("merged image: {} ({} bytes)", bin.display(), image.len());
    println!("intel hex:    {}", hex.display());
}

/// Converts an ELF to a flat binary with `llvm-objcopy`.
fn flatten(elf: &Path, core: &Core) -> Vec<u8> {
    if !elf.exists() {
        fail(&format!(
            "{} is not built — run `cargo xtask build --example {}`",
            elf.display(),
            elf.file_stem().unwrap_or_default().to_string_lossy()
        ));
    }
    let tmp = env::temp_dir().join(format!("h417-{}-{}.bin", core.name, std::process::id()));
    let objcopy = objcopy();
    run(
        &objcopy,
        &["-O", "binary", elf.to_str().unwrap(), tmp.to_str().unwrap()],
        &root(),
    );
    let bytes =
        fs::read(&tmp).unwrap_or_else(|e| fail(&format!("cannot read {}: {e}", tmp.display())));
    let _ = fs::remove_file(&tmp);

    if bytes.len() as u64 > core.flash_limit {
        fail(&format!(
            "{}: {} bytes exceeds its {} byte flash region — it would overlap the other image",
            core.name,
            bytes.len(),
            core.flash_limit
        ));
    }
    bytes
}

/// `llvm-objcopy` shipped with the active toolchain, falling back to `PATH`.
fn objcopy() -> String {
    let output = |args: &[&str]| {
        Command::new("rustc")
            .args(args)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };
    if let (Some(sysroot), Some(host)) = (output(&["--print", "sysroot"]), output(&["-vV"])) {
        let host = host
            .lines()
            .find_map(|line| line.strip_prefix("host: ").map(str::to_string));
        if let Some(host) = host {
            let path = Path::new(&sysroot)
                .join("lib/rustlib")
                .join(host)
                .join("bin/llvm-objcopy");
            if path.exists() {
                return path.to_string_lossy().into_owned();
            }
        }
    }
    "llvm-objcopy".to_string()
}

/// Intel HEX for the payload segments only — the erased padding between the two
/// images would otherwise be tens of thousands of pointless `0xFF` records.
fn intel_hex(segments: &[Segment]) -> String {
    let mut hex = String::new();
    let mut upper = u32::MAX;
    for segment in segments {
        for (i, chunk) in segment.bytes.chunks(16).enumerate() {
            let address = segment.base as u32 + (i * 16) as u32;
            if address >> 16 != upper {
                upper = address >> 16;
                hex.push_str(&record(0x04, 0, &[(upper >> 8) as u8, upper as u8]));
            }
            hex.push_str(&record(0x00, (address & 0xFFFF) as u16, chunk));
        }
    }
    hex.push_str(":00000001FF\n");
    hex
}

/// One Intel HEX record: length, 16-bit address, type, data, checksum.
///
/// The length is derived from `data` — passing it in separately is how the
/// extended-address records ended up claiming zero bytes, which made `wlink`
/// reject the whole file with "payload length does not match record header".
fn record(record_type: u8, address: u16, data: &[u8]) -> String {
    let mut bytes = vec![
        data.len() as u8,
        (address >> 8) as u8,
        address as u8,
        record_type,
    ];
    bytes.extend_from_slice(data);
    let sum = bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b));
    let checksum = (!sum).wrapping_add(1);

    let mut line = format!(":{bytes:02X?}");
    line.retain(|c| !matches!(c, '[' | ']' | ' ' | ','));
    format!("{line}{checksum:02X}\n")
}

/// `v3f` / `v3f+v5f` label for the final build line.
fn describe(cores: &[&'static Core]) -> String {
    cores
        .iter()
        .map(|core| core.name)
        .collect::<Vec<_>>()
        .join("+")
}

fn fail(message: &str) -> ! {
    eprintln!("xtask: {message}");
    exit(1);
}
