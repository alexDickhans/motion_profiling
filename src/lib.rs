#![no_std]
extern crate alloc;

pub mod path;
pub mod motion_profile;
pub mod mp_2d;
pub mod combined_mp;

#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
        assert_eq!(4, 4);
    }
}
