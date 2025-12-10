"""Unit tests for Edge Deploy Skill."""

import pytest
from skills.edge_deploy.src.main import EdgedeploySkill


class TestEdgedeploySkill:
    def setup_method(self):
        self.skill = EdgedeploySkill()

    def test_init(self):
        assert self.skill is not None

    def test_execute(self):
        result = self.skill.execute()
        assert result["status"] == "success"
