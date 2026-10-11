//! 共享内存区域封装 (基于 memmap2)
//!
//! 使用内存映射文件替代 POSIX 共享内存，纯 Rust 实现。

use memmap2::MmapMut;
use std::fs::OpenOptions;
use std::io::{self, Write};
use std::path::PathBuf;

#[derive(Debug)]
pub enum ShmError {
    IoError(io::Error),
    MapFailed,
    InvalidSize,
}

impl std::fmt::Display for ShmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ShmError::IoError(e) => write!(f, "IO error: {}", e),
            ShmError::MapFailed => write!(f, "mmap failed"),
            ShmError::InvalidSize => write!(f, "invalid size"),
        }
    }
}

impl std::error::Error for ShmError {}

impl From<io::Error> for ShmError {
    fn from(e: io::Error) -> Self {
        ShmError::IoError(e)
    }
}

/// 获取共享内存文件路径
fn shm_path(name: &str) -> PathBuf {
    // Prefer TALOS_SHM_DIR so Windows host + WSL can share the same folder
    // (e.g. F:\talos_shm on Windows, /mnt/f/talos_shm in WSL).
    let clean_name = name.trim_start_matches('/');
    let base = std::env::var_os("TALOS_SHM_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            // On Windows, bare "/tmp" is drive-relative and unreliable.
            if cfg!(windows) {
                std::env::temp_dir().join("talos_shm")
            } else {
                PathBuf::from("/tmp")
            }
        });
    let _ = std::fs::create_dir_all(&base);
    base.join(clean_name)
}

/// RAII 封装的共享内存区域
pub struct ShmRegion {
    mmap: MmapMut,
    path: PathBuf,
    is_owner: bool,
}

unsafe impl Send for ShmRegion {}
unsafe impl Sync for ShmRegion {}

impl ShmRegion {
    /// 创建新的共享内存区域 (生产者)
    pub fn create(name: &str, size: usize) -> Result<Self, ShmError> {
        let path = shm_path(name);

        // 创建或覆盖文件
        let mut file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&path)?;

        // 设置文件大小
        file.set_len(size as u64)?;

        // 写入零填充
        file.write_all(&vec![0u8; size])?;
        file.sync_all()?;

        // 重新打开并映射
        let file = OpenOptions::new().read(true).write(true).open(&path)?;

        let mmap = unsafe { MmapMut::map_mut(&file)? };

        Ok(Self {
            mmap,
            path,
            is_owner: true,