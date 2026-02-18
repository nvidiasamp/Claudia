#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pcm_utils.py 重采样函数のユニットテスト"""

import sys
import os
import unittest

import numpy as np

# プロジェクトパス追加
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "src"))

from claudia.audio.pcm_utils import resample_pcm_int16


class TestResamplePcmInt16(unittest.TestCase):
    """resample_pcm_int16 テスト"""

    def test_same_rate_noop(self):
        """同一レート時は入力をそのまま返す"""
        samples = np.array([1, 2, 3, 4], dtype=np.int16)
        result = resample_pcm_int16(samples, 16000, 16000)
        np.testing.assert_array_equal(result, samples)

    def test_downsample_44100_to_16000(self):
        """44100→16000 ダウンサンプリング: 出力長が正しい"""
        n_in = 44100  # 1秒分
        samples = np.arange(n_in, dtype=np.int16)
        result = resample_pcm_int16(samples, 44100, 16000)
        expected_len = int(n_in * 16000 / 44100)
        self.assertEqual(len(result), expected_len)
        self.assertEqual(result.dtype, np.int16)

    def test_upsample_16000_to_44100(self):
        """16000→44100 アップサンプリング: 出力長が正しい"""
        n_in = 16000  # 1秒分
        samples = np.arange(n_in, dtype=np.int16)
        result = resample_pcm_int16(samples, 16000, 44100)
        expected_len = int(n_in * 44100 / 16000)
        self.assertEqual(len(result), expected_len)

    def test_output_within_input_range(self):
        """出力値が入力値の範囲内に収まる (近傍補間)"""
        samples = np.array([0, 100, 200, 300, 32767], dtype=np.int16)
        result = resample_pcm_int16(samples, 44100, 16000)
        self.assertTrue(np.all(result >= 0))
        self.assertTrue(np.all(result <= 32767))

    def test_empty_input(self):
        """空配列は空を返す"""
        samples = np.array([], dtype=np.int16)
        result = resample_pcm_int16(samples, 44100, 16000)
        self.assertEqual(len(result), 0)

    def test_backward_compat_asr_service_import(self):
        """asr_service パッケージからの re-export が機能する"""
        from claudia.audio.asr_service import resample_pcm_int16 as fn
        self.assertIs(fn, resample_pcm_int16)


if __name__ == "__main__":
    unittest.main()
