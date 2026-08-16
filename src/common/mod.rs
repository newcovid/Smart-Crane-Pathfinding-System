// common/common.rs 与父模块同名。保留该布局是因为 common.rs 存放的是
// 真正的共享类型（Node / FlatGrid / 网格解析），而 constants.rs 是纯常量，
// 拆开比合并成一个大文件更清晰。
#![allow(clippy::module_inception)]

pub mod common;
pub mod constants;

// 将子模块 common 中的所有公开项（Node, FlatGrid等）提升到 crate::common::*
pub use common::*;
// 将常量提升到 crate::common::* 方便直接访问
pub use constants::*;
