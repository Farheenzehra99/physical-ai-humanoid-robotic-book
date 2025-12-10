"""Unit tests for Unity Vis Skill."""

import pytest
from skills.unity_vis.src.main import UnityvisSkill


class TestUnityvisSkill:
    def setup_method(self):
        self.skill = UnityvisSkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
