#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCM 音频重采样工具

Tegra ALSA plughw 在 AT2020USB-XP 等 USB 麦克风上重采样会产出全零，
必须在 Python 层做。此模块提供共享的重采样函数，供 asr_server 和
audio_capture 共用。
"""


def resample_pcm_int16(samples_int16, src_rate: int, dst_rate: int):
    """PCM int16 重采样 (numpy 线性索引)

    Parameters
    ----------
    samples_int16 : np.ndarray (int16)
        源采样率的 int16 PCM 样本
    src_rate : int
        源采样率 (如 44100)
    dst_rate : int
        目标采样率 (如 16000)

    Returns
    -------
    np.ndarray (int16)
        重采样后的 int16 PCM
    """
    if src_rate == dst_rate:
        return samples_int16
    import numpy as np
    n_out = int(len(samples_int16) * dst_rate / src_rate)
    indices = (np.arange(n_out, dtype=np.float64) * src_rate / dst_rate).astype(np.int64)
    indices = np.clip(indices, 0, len(samples_int16) - 1)
    return samples_int16[indices]
