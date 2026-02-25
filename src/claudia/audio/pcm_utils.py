#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCM Audio Resampling Utility

Tegra ALSA plughw produces all-zero output when resampling USB microphones
like the AT2020USB-XP, so resampling must be done in the Python layer.
This module provides a shared resampling function used by both asr_server
and audio_capture.
"""


def resample_pcm_int16(samples_int16, src_rate: int, dst_rate: int):
    """PCM int16 resampling (numpy linear indexing)

    Parameters
    ----------
    samples_int16 : np.ndarray (int16)
        int16 PCM samples at the source sample rate
    src_rate : int
        Source sample rate (e.g., 44100)
    dst_rate : int
        Target sample rate (e.g., 16000)

    Returns
    -------
    np.ndarray (int16)
        Resampled int16 PCM
    """
    if src_rate == dst_rate:
        return samples_int16
    import numpy as np
    n_out = int(len(samples_int16) * dst_rate / src_rate)
    indices = (np.arange(n_out, dtype=np.float64) * src_rate / dst_rate).astype(np.int64)
    indices = np.clip(indices, 0, len(samples_int16) - 1)
    return samples_int16[indices]
