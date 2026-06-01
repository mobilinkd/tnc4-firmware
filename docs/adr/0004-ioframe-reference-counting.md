# 0004: IoFrame reference counting

Replaced single-owner `acquire()/release()` with reference counting. Audit of all 50+ existing call sites confirmed correctness — every `acquire()` is paired with exactly one `release()`, and the ref-counted `release()` is a behavioral superset.

See `Core/TNC/HdlcFrame.hpp` Frame::ref_count_, FramePool::add_ref(), FramePool::release().
