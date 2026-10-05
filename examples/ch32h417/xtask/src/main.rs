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
//! cargo xtask build [--example NAME] [--v3f-only]
//! cargo xtask flash --example NAME [--v3f-only] [--no-build]
//! cargo xtask run   --example NAME [--v3f-only] [--no-build]
//! cargo xtask merge --example NAME [--out DIR]
//! cargo xtask report
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
    /// Directory name below this one.
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

const USAGE: &str = "\
usage: cargo xtask <command> [options]

commands:
  build [--example NAME] [--dual-core | --jump-v5f]     build one example (or all)
  flash --example NAME [--dual-core | --jump-v5f] [--no-build]
                                            build, then flash and reset
  run   --example NAME [--dual-core | --jump-v5f] [--no-build]
                                            like flash, plus the SDI console
  merge --example NAME [--dual-core | --jump-v5f] [--out DIR]
                                            merge the images into .bin + .hex
  report                                    read the mailbox back as a report

what gets programmed:
  (default)      the boot core alone (same as --v3f-only); the V5F region keeps
                 whatever image it already holds
  --dual-core    both halves of the named example, V3F first
  --jump-v5f     the generic jump-only firmware (`launcher`) on the boot core
                 and, when --example names one, that example's V5F half

Reset only ever starts the V3F, so there is no V5F-only mode; see the module
docs.";

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
        "report" => report(),
        "help" | "--help" | "-h" => println!("{USAGE}"),
        other => fail(&format!("unknown command `{other}`\n\n{USAGE}")),
    }
}

/// Parsed command line.
struct Options {
    example: Option<String>,
    layout: Layout,
    no_build: bool,
    out: Option<PathBuf>,
}

impl Options {
    fn parse(args: &[String]) -> Options {
        let mut opts = Options {
            example: None,
            layout: Layout::V3fOnly,
            no_build: false,
            out: None,
        };
        let mut args = args.iter();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--example" => opts.example = args.next().cloned(),
                // The default, accepted so a script can say what it means.
                "--v3f-only" => opts.layout = Layout::V3fOnly,
                "--dual-core" => opts.layout = Layout::DualCore,
                "--jump-v5f" => opts.layout = Layout::JumpV5f,
                "--out" => opts.out = args.next().map(PathBuf::from),
                "--no-build" => opts.no_build = true,
                other => fail(&format!("unknown option `{other}`\n\n{USAGE}")),
            }
        }
        opts
    }

    /// Cores to act on, each with the image that supplies it.
    ///
    /// With no `--example` this is every core, which is what `build` wants; the
    /// commands that need an image of their own reject the empty example.
    fn jobs(&self) -> Vec<Job> {
        let job = |core: &'static Core, example: &str| Job {
            core,
            example: example.to_string(),
        };
        let boot = core_named(BOOT_CORE);
        let second = core_named(SECOND_CORE);

        match (self.example.as_deref(), self.layout) {
            (None, Layout::JumpV5f) => vec![job(boot, JUMP_FIRMWARE)],
            (None, _) => CORES
                .iter()
                .map(|core| Job {
                    core,
                    example: String::new(),
                })
                .collect(),
            // The boot core alone; the V5F region keeps whatever it holds.
            (Some(example), Layout::V3fOnly) => {
                require_source(boot, example);
                vec![job(boot, example)]
            }
            (Some(example), Layout::DualCore) => {
                require_source(boot, example);
                require_source(second, example);
                vec![job(boot, example), job(second, example)]
            }
            // The generic jumper on the boot core, the named example on hart 1.
            (Some(example), Layout::JumpV5f) => {
                require_source(second, example);
                vec![job(boot, JUMP_FIRMWARE), job(second, example)]
            }
        }
    }
}

/// What to program.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Layout {
    /// The named example's boot-core half only. The default: the boot core is
    /// what reset starts, and rewriting the V5F payload is opt-in.
    V3fOnly,
    /// Both halves of the named example.
    DualCore,
    /// The generic jump-only firmware on the boot core, plus the named
    /// example's V5F half when an example is given.
    JumpV5f,
}

/// One core to act on, and the example that supplies its image.
struct Job {
    core: &'static Core,
    example: String,
}

/// The core reset starts, and the one that carries the entry point.
const BOOT_CORE: &str = "v3f";

/// The other core, which only ever runs what the boot core hands over to it.
const SECOND_CORE: &str = "v5f";

/// The V3F-only firmware whose whole job is to hand over to hart 1.
const JUMP_FIRMWARE: &str = "launcher";

/// The core with this directory name.
fn core_named(name: &str) -> &'static Core {
    CORES
        .iter()
        .find(|core| core.name == name)
        .expect("core names are fixed")
}

/// Fails unless `core` carries `example`.
fn require_source(core: &Core, example: &str) {
    if !example_source(core, example).exists() {
        fail(&format!(
            "`{example}` has no {}-core half ({} does not exist)",
            core.name,
            example_source(core, example).display()
        ));
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
    run_env(program, args, dir, &[])
}

/// Like [`run`], with extra environment variables for the child.
fn run_env(program: &str, args: &[&str], dir: &Path, env: &[(&str, &str)]) {
    println!("  {} {}", program, args.join(" "));
    let status = Command::new(program)
        .args(args)
        .current_dir(dir)
        .envs(env.iter().copied())
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
    let jobs = opts.jobs();
    // The boot core owns the shared region, so it is built first; its ELF is then
    // post-processed into the layout the other cores consume.
    let mut layout: Option<PathBuf> = None;

    for job in &jobs {
        if job.core.name != BOOT_CORE && layout.is_none() {
            layout = Some(write_layout(&jobs));
        }

        let dir = root().join(job.core.name);
        let mut args = vec!["build", "--release"];
        if !job.example.is_empty() {
            args.extend(["--bin", &job.example]);
        }
        println!("building {} ...", job.core.name);

        match (&layout, job.core.name != BOOT_CORE) {
            (Some(path), true) => run_env(
                "cargo",
                &args,
                &dir,
                &[("H417_LAYOUT", path.to_str().expect("utf-8 path"))],
            ),
            _ => run("cargo", &args, &dir),
        }
    }
    if opts.example.is_some() {
        println!("built: {}", describe(&jobs));
    }
}

/// Post-processes the boot core's ELF into the shared layout the other cores
/// consume: the address and size its linker gave to the mailbox, plus the layout
/// version. Written as plain `key 0xvalue` lines so `v5f/build.rs` can turn it
/// into constants and `report` can read it back.
fn write_layout(jobs: &[Job]) -> PathBuf {
    let boot = jobs
        .iter()
        .find(|job| job.core.name == BOOT_CORE)
        .expect("a boot-core job");
    let candidates: Vec<String> = if boot.example.is_empty() {
        // `cargo xtask build` with no example: any boot-core image will do, the
        // region is a property of the map, not of one example.
        ["dualcore", "cpuid", "sdi_cpuid", "launcher"]
            .iter()
            .map(|name| name.to_string())
            .collect()
    } else {
        vec![boot.example.clone()]
    };

    for example in candidates {
        let elf = example_elf(core_named(BOOT_CORE), &example);
        if !elf.exists() {
            continue;
        }
        if let Some((address, size)) = mailbox_symbol(&elf) {
            let out_dir = root().join("out");
            fs::create_dir_all(&out_dir)
                .unwrap_or_else(|e| fail(&format!("cannot create {}: {e}", out_dir.display())));
            let path = out_dir.join("layout.txt");
            let text = format!(
                "mailbox_addr {address:#010x}
mailbox_size {size:#x}
layout_version {}
",
                ch32h417_ipc::LAYOUT_VERSION
            );
            fs::write(&path, text)
                .unwrap_or_else(|e| fail(&format!("cannot write {}: {e}", path.display())));
            println!(
                "layout: {} -> {address:#010x} ({size} bytes)",
                path.display()
            );
            return path;
        }
    }
    fail(
        "no boot-core image with a `MAILBOX` symbol found — build a boot-core example first\n\
         \x20     (all shared symbols are defined by the V3F image; the V5F consumes their address)",
    )
}

/// `(address, size)` of the `MAILBOX` symbol in `elf`.
fn mailbox_symbol(elf: &Path) -> Option<(u32, u32)> {
    let output = Command::new(nm())
        .args(["-S", elf.to_str()?])
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&output.stdout);
    for line in text.lines() {
        let mut fields = line.split_whitespace();
        let (address, size) = (fields.next()?, fields.next()?);
        if !line.ends_with("MAILBOX") {
            continue;
        }
        return Some((
            u32::from_str_radix(address, 16).ok()?,
            u32::from_str_radix(size, 16).ok()?,
        ));
    }
    None
}

/// `flash` (watch = false) and `run` (watch = true) share this.
///
/// The non-final cores are written with `--no-run` and read back, so nothing
/// starts half-programmed. The final write resets and runs the chip and carries
/// the SDI flags — and nothing may follow it, because *every* other wlink
/// request (`dump`, `regs`, `status`) pauses the cores: a read-back check there
/// would measure the paused state instead of the running one.
fn flash(opts: &Options, watch: bool) {
    let jobs = opts.jobs();
    if jobs.iter().any(|job| job.example.is_empty()) {
        fail(&format!(
            "`{}` needs --example (or `--jump-v5f` on its own)\n\n{USAGE}",
            if watch { "run" } else { "flash" }
        ));
    }
    if !opts.no_build {
        build(opts);
    }
    if opts.layout == Layout::JumpV5f {
        if let Some(example) = &opts.example {
            if example_source(core_named(BOOT_CORE), example).exists() {
                println!(
                    "note: --jump-v5f replaces `{example}`'s own V3F half with the jumper,\n\
                     \x20     so whatever that half printed or drove will not happen. Use\n\
                     \x20     --dual-core when the example's V3F half does real work\n\
                     \x20     (e.g. `sdi_cpuid`'s print-token handshake)."
                );
            }
        }
    }
    let writes_second_core = jobs.iter().any(|job| job.core.name != BOOT_CORE);

    for (i, job) in jobs.iter().enumerate() {
        let core = job.core;
        let elf = example_elf(core, &job.example);
        let last = i + 1 == jobs.len();
        println!("flashing {} ({}) ...", core.name, job.example);

        if last {
            // The final write resets and runs the chip. `run` adds the SDI pair
            // (`--enable-sdi-print --watch-serial`: the console needs the target
            // running, which is also what keeps hart 1 alive); `flash` stays
            // plain, because enabling SDI print without watching leaves the
            // debug module attached and the second core paused.
            let mut args = vec!["flash"];
            if watch {
                if writes_second_core {
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
            program_verified(core, &job.example, &elf, &args, true);
        }
    }
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
            // image-sized prefix rather than the vectors themselves, and treat
            // the bytes the image leaves as erased as don't-care: sections are
            // written individually, so the alignment gaps between them keep
            // whatever the flash held there.
            Some(actual) if actual.len() >= expected.len() => {
                let wrong = expected
                    .iter()
                    .zip(&actual)
                    .filter(|(want, got)| **want != ERASED && want != got)
                    .count();
                if wrong == 0 {
                    println!("  {} verified ({} bytes)", core.name, expected.len());
                    return;
                }
                println!(
                    "  {}: read-back differs in {wrong} of {} bytes on attempt {attempt}",
                    core.name,
                    expected.len()
                );
            }
            Some(_) => println!("  {}: read-back was short on attempt {attempt}", core.name),
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
    let jobs = opts.jobs();
    let boot = jobs.iter().find(|job| job.core.name == BOOT_CORE && !job.example.is_empty());
    let second = jobs
        .iter()
        .find(|job| job.core.name == SECOND_CORE && !job.example.is_empty());
    let (Some(boot), Some(second)) = (boot, second) else {
        fail(&format!(
            "merging needs both cores — pass --dual-core, or --jump-v5f with --example\n\n{USAGE}"
        ));
    };

    let segments: Vec<Segment> = [boot, second]
        .into_iter()
        .map(|job| Segment {
            core: job.core.name,
            base: job.core.flash_base,
            bytes: flatten(&example_elf(job.core, &job.example), job.core),
        })
        .collect();
    // The merged file is named after what runs on hart 1, with the jumper called
    // out when the boot core is the generic firmware.
    let name = if boot.example == JUMP_FIRMWARE {
        format!("jump-{}", second.example)
    } else {
        second.example.clone()
    };

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
    let bin = out_dir.join(format!("{name}.bin"));
    let hex = out_dir.join(format!("{name}.hex"));
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

/// The mailbox address the boot core exported, as written by [`write_layout`].
fn read_layout_address() -> u64 {
    let path = root().join("out/layout.txt");
    let text = fs::read_to_string(&path).unwrap_or_else(|e| {
        fail(&format!(
            "cannot read {}: {e}\n     run `cargo xtask build --example NAME --dual-core` first",
            path.display()
        ))
    });
    for line in text.lines() {
        if let Some(raw) = line.strip_prefix("mailbox_addr ") {
            if let Ok(address) = u32::from_str_radix(raw.trim().trim_start_matches("0x"), 16) {
                return address as u64;
            }
        }
    }
    fail(&format!("no mailbox_addr in {}", path.display()))
}

/// Reads the mailbox back and decodes it.
///
/// The V5F half cannot report over SDI while a console is attached — every wlink
/// request pauses the cores, and the console keeps one attached — so the examples
/// publish what they find in shared RAM instead, and this prints it afterwards.
///
/// The layout comes from the `ch32h417-ipc` crate: the very `#[repr(C)]` struct
/// both cores place at `SRAM_SHARED`, linked into this tool so the field offsets
/// decoded here cannot drift from the ones they write.
fn report() {
    /// Offset of one mailbox field, from the shared definition.
    macro_rules! at {
        ($field:ident) => {
            core::mem::offset_of!(ch32h417_ipc::Mailbox, $field)
        };
    }

    /// Bytes to read: the whole structure, so a field added at the end cannot be
    /// decoded past the end of the dump.
    const LEN: usize = core::mem::size_of::<ch32h417_ipc::Mailbox>();

    let base = read_layout_address();
    let tmp = env::temp_dir().join(format!("h417-mailbox-{}.bin", std::process::id()));
    let address = format!("{base:#010x}");
    let length = LEN.to_string();
    let read = run_timed(
        "wlink",
        &[
            "dump",
            &address,
            &length,
            "-o",
            tmp.to_str().expect("utf-8 path"),
        ],
        &root(),
        QUICK_TIMEOUT,
    );
    if !read {
        fail("could not read the mailbox");
    }
    let bytes =
        fs::read(&tmp).unwrap_or_else(|e| fail(&format!("reading {}: {e}", tmp.display())));
    let _ = fs::remove_file(&tmp);

    let word = |offset: usize| -> u32 {
        u32::from_le_bytes([bytes[offset], bytes[offset + 1], bytes[offset + 2], bytes[offset + 3]])
    };

    println!("mailbox @ {base:#010x}");
    let version = word(at!(layout_version));
    if version != ch32h417_ipc::LAYOUT_VERSION {
        println!(
            "  note: the flashed half reports mailbox layout v{version}, this tool knows v{} — \
             reflash both halves",
            ch32h417_ipc::LAYOUT_VERSION
        );
    }

    println!("  sdi_cpuid token   = {:#010x}", word(at!(sdi_token)));
    println!("  sdi_cpuid ticks   = {}", word(at!(sdi_ticks)));
    let marker = word(at!(dualcore_marker));
    println!(
        "  dualcore  marker  = {marker:#010x}{}",
        if marker == ch32h417_ipc::DUALCORE_MARKER {
            "  (hart 1 ran)"
        } else {
            ""
        }
    );
    println!("  dualcore  counter = {}", word(at!(dualcore_counter)));
    println!("  pingpong  ping    = {}", word(at!(ping)));
    println!("  pingpong  pong    = {}", word(at!(pong)));

    if word(at!(cpuid_done)) != ch32h417_ipc::CPUID_DONE {
        println!(
            "  cpuid: no report from hart 1 (magic missing, progress = {})",
            word(at!(cpuid_progress))
        );
        return;
    }
    println!(
        "  cpuid  (hart 1)   image build features = {:#04x}",
        word(at!(cpuid_buildcfg))
    );
    let present = word(at!(cpuid_present));
    let mut misa = None;
    for (i, name) in ch32h417_ipc::CPUID_CSR_NAMES.iter().enumerate() {
        let value = word(at!(cpuid_values) + i * 4);
        if present & (1 << i) == 0 {
            println!("    {name:<18}   absent (read faulted)");
            continue;
        }
        println!("    {name:<18} = {value:#010x}");
        if i == ch32h417_ipc::CPUID_MISA_INDEX {
            misa = Some(value);
        }
    }
    if let Some(misa) = misa {
        let letters: String = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            .chars()
            .enumerate()
            .filter(|(bit, _)| misa & (1 << bit) != 0)
            .map(|(_, letter)| letter)
            .collect();
        println!(
            "    misa decode        MXL={} extensions={letters}",
            (misa >> 30) & 3
        );
    }
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
    // `--gap-fill 0xFF` makes the gaps between sections read as erased flash,
    // which is what is actually there: the sections are written individually.
    run(
        &objcopy,
        &[
            "-O",
            "binary",
            "--gap-fill",
            "0xFF",
            elf.to_str().unwrap(),
            tmp.to_str().unwrap(),
        ],
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
/// `llvm-nm` shipped with the active toolchain, falling back to `PATH`.
fn nm() -> String {
    tool("llvm-nm")
}

/// `llvm-objcopy` shipped with the active toolchain, falling back to `PATH`.
fn objcopy() -> String {
    tool("llvm-objcopy")
}

fn tool(name: &str) -> String {
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
                .join("bin")
                .join(name);
            if path.exists() {
                return path.to_string_lossy().into_owned();
            }
        }
    }
    name.to_string()
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

/// `v3f:cpuid+v5f:cpuid` label for the final build line.
fn describe(jobs: &[Job]) -> String {
    jobs.iter()
        .map(|job| {
            if job.example.is_empty() {
                job.core.name.to_string()
            } else {
                format!("{}:{}", job.core.name, job.example)
            }
        })
        .collect::<Vec<_>>()
        .join("+")
}

fn fail(message: &str) -> ! {
    eprintln!("xtask: {message}");
    exit(1);
}
