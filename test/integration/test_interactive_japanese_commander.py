#!/usr/bin/env python3
"""
Production Brain Integration Test — 端到端管道验证

验证 ProductionBrain 的完整处理流程:
  用户输入 → emergency/cache/LLM → SafetyCompiler → 执行/模拟
不依赖 Ollama、ROS2、CycloneDDS 或真实硬件（全部 mock）。
"""

import asyncio
import os
import sys
import unittest
from unittest.mock import MagicMock

# 在 import ProductionBrain 之前 mock 重量级依赖，避免 OOM
# ollama: 避免加载 LLM 运行时
sys.modules.setdefault('ollama', MagicMock())
# cyclonedds: 避免 C 库 bad_alloc（DDS 网络初始化）
for mod in ['cyclonedds', 'cyclonedds.core', 'cyclonedds.domain',
            'cyclonedds.idl', 'cyclonedds.pub', 'cyclonedds.sub',
            'cyclonedds._clayer']:
    sys.modules.setdefault(mod, MagicMock())


class TestProductionBrainIntegration(unittest.TestCase):
    """ProductionBrain 集成测试 — 模拟模式"""

    @classmethod
    def setUpClass(cls):
        os.environ["BRAIN_ROUTER_MODE"] = "legacy"
        os.environ["BRAIN_MODEL_7B"] = "test-model-7b"

        from claudia.brain.production_brain import ProductionBrain
        cls._brain_cls = ProductionBrain

    def setUp(self):
        self.brain = self._brain_cls(use_real_hardware=False)

        async def _mock_ensure(model, **kwargs):
            return True
        self.brain._ensure_model_loaded = _mock_ensure

    def _run(self, coro):
        """同步执行异步协程"""
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(coro)
        finally:
            loop.close()

    # ------------------------------------------------------------------
    # Emergency bypass
    # ------------------------------------------------------------------

    def test_emergency_stop(self):
        r = self._run(self.brain.process_and_execute("止まれ"))
        self.assertEqual(r.api_code, 1003)
        self.assertEqual(r.execution_status, "success")

    def test_emergency_with_punctuation(self):
        r = self._run(self.brain.process_and_execute("止まって！"))
        self.assertEqual(r.api_code, 1003)

    def test_emergency_kana(self):
        r = self._run(self.brain.process_and_execute("とまれ"))
        self.assertEqual(r.api_code, 1003)

    def test_emergency_english(self):
        r = self._run(self.brain.process_and_execute("stop"))
        self.assertEqual(r.api_code, 1003)

    # ------------------------------------------------------------------
    # Hot cache
    # ------------------------------------------------------------------

    def test_cache_sit(self):
        r = self._run(self.brain.process_and_execute("座って"))
        self.assertEqual(r.api_code, 1009)

    def test_cache_stand(self):
        r = self._run(self.brain.process_and_execute("立って"))
        self.assertEqual(r.api_code, 1004)

    def test_cache_kana_alias(self):
        r = self._run(self.brain.process_and_execute("たって"))
        self.assertEqual(r.api_code, 1004)

    # ------------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------------

    def test_safety_blocks_high_risk(self):
        async def mock_llm(model, command, timeout=30, **kw):
            return {"r": "前転します", "a": 1030}
        self.brain._call_ollama_v2 = mock_llm

        r = self._run(self.brain.process_and_execute("前転して"))
        self.assertNotEqual(r.api_code, 1030)


if __name__ == "__main__":
    unittest.main()
