
use core::fmt::Write as CoreWrite;


/// This is a very simple buffer to pre format a short line of text
/// limited arbitrarily to 64 bytes.
pub struct FmtBuf {
    buf: [u8; 64],
    ptr: usize,
}

impl FmtBuf {
    pub fn new() -> Self {
        Self {
            buf: [0; 64],
            ptr: 0,
        }
    }

    pub fn reset(&mut self) {
        self.ptr = 0;
    }
    
    #[allow(unused)]
    pub fn copy(&mut self, chars: &[u8]) {
        self.ptr = chars.len();
        self.buf[0..self.ptr].copy_from_slice(chars);
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.buf[0..self.ptr]).unwrap()
    }
}
impl Copy for FmtBuf {}

impl Clone for FmtBuf{
    fn clone(&self) -> Self {
        Self { buf: self.buf.clone(), ptr: self.ptr.clone() }
    }
}

impl CoreWrite for FmtBuf {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let rest_len = self.buf.len() - self.ptr;
        let len = if rest_len < s.len() {
            rest_len
        } else {
            s.len()
        };
        self.buf[self.ptr..(self.ptr + len)].copy_from_slice(&s.as_bytes()[0..len]);
        self.ptr += len;
        Ok(())
    }
}
