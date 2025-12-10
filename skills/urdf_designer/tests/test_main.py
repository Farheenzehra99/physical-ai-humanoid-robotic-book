"""Unit tests for URDF Designer Skill."""

import pytest
from skills.urdf_designer.src.main import URDFDesignerSkill


class TestURDFDesignerSkill:
    def setup_method(self):
        self.skill = URDFDesignerSkill()

    def test_generate_humanoid_urdf(self):
        result = self.skill.generate_humanoid_urdf("test_robot", 18, 1.2)
        assert "urdf_path" in result
        assert result["status"] == "generated"

    def test_validate_urdf(self):
        result = self.skill.validate_urdf("/tmp/test.urdf")
        assert result["valid"] is True
