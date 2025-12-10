"""Unit tests for Hardware Proxy Skill."""

import pytest
from skills.hardware_proxy.src.main import HardwareproxySkill


class TestHardwareproxySkill:
    def setup_method(self):
        self.skill = HardwareproxySkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
