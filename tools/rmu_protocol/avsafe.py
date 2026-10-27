"""PyAV 像素格式转换的线程安全包装。

PyAV 17 / FFmpeg 8 在 Windows 上，多个线程同时调用 VideoFrame.reformat()（含 to_image/from_ndarray
后的隐式转换）会出问题，两种都实测复现过：
  * 默认 threads=0：swscale 的 slice 线程池偶发死锁，线程永久停在 avpriv_slicethread_execute
    （Mock 跑几百到几千帧后 3334 停帧，0x0310 队列持续积压）；
  * threads=1：两线程并发时直接 access violation 崩进程。
所以所有转换都经过这把进程级锁串行执行。单次 1280×720 转换几 ms，30 fps + 15 fps 两路远低于饱和。
"""
from __future__ import annotations

import threading

import av

_LOCK = threading.Lock()


def reformat(frame: av.VideoFrame, **kwargs) -> av.VideoFrame:
    with _LOCK:
        return frame.reformat(**kwargs)


def to_image(frame: av.VideoFrame):
    with _LOCK:
        return frame.to_image()


def from_ndarray(array, format: str) -> av.VideoFrame:
    with _LOCK:
        return av.VideoFrame.from_ndarray(array, format=format)


def from_image(image) -> av.VideoFrame:
    with _LOCK:
        return av.VideoFrame.from_image(image)
