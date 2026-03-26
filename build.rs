fn main() {
    #[cfg(feature = "rt")]
    {
        use imxrt_rt::{Family, FlexRamBanks, Memory, RuntimeBuilder};

        RuntimeBuilder::from_flexspi(Family::Imxrt1060, 1984 * 1024)
            .flexram_banks(FlexRamBanks {
                ocram: 0,
                itcm: 6,
                dtcm: 10,
            })
            .heap(Memory::Ocram)
            .heap_size(16 * 1024)
            .heap_size_env_override("TEENSY4_HEAP_SIZE")
            .stack(Memory::Dtcm)
            .stack_size(16 * 1024)
            .stack_size_env_override("TEENSY4_STACK_SIZE")
            .vectors(Memory::Dtcm)
            .text(Memory::Itcm)
            .data(Memory::Dtcm)
            .bss(Memory::Dtcm)
            .uninit(Memory::Ocram)
            .linker_script_name("t4link.x")
            .build()
            .unwrap();
    }

    #[cfg(feature = "psram")]
    {
        let out_dir = std::env::var("OUT_DIR").unwrap();
        let psram_x = std::path::Path::new(&out_dir).join("psram.x");
        std::fs::copy("psram.x", &psram_x).unwrap();
        // Cargo appends rustc-link-arg after RUSTFLAGS, so t4link.x
        // (from .cargo/config.toml) is processed first, ensuring FLASH
        // and .uninit are defined before psram.x references them.
        println!("cargo:rustc-link-arg=-Tpsram.x");
        println!("cargo:rerun-if-changed=psram.x");
    }
}
