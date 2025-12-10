"""Unit tests for Gazebo Sim Skill."""

import pytest
from skills.gazebo_sim.src.main import GazebosimSkill


class TestGazebosimSkill:
    def setup_method(self):
        self.skill = GazebosimSkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
