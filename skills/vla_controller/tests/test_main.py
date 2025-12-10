"""Unit tests for Vla Controller Skill."""

import pytest
from skills.vla_controller.src.main import VlacontrollerSkill


class TestVlacontrollerSkill:
    def setup_method(self):
        self.skill = VlacontrollerSkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
