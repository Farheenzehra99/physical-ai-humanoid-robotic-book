"""Unit tests for Isaac Sim Pipeline Skill."""

import pytest
from skills.isaac_sim_pipeline.src.main import IsaacsimpipelineSkill


class TestIsaacsimpipelineSkill:
    def setup_method(self):
        self.skill = IsaacsimpipelineSkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
