"""Unit tests for sim_agent workflows."""

import pytest
from unittest.mock import Mock, patch
from agents.sim_agent.workflows.base import BaseWorkflow


class TestBaseWorkflow:
    def setup_method(self):
        # Mock route config path
        self.config_path = "../../specs/001-physical-ai-humanoid/contracts/agents/sim_agent-routes.json"

    @patch("builtins.open")
    @patch("json.load")
    def test_init(self, mock_json_load, mock_open):
        mock_json_load.return_value = {"routes": []}
        workflow = BaseWorkflow(self.config_path)
        assert workflow.routes == {"routes": []}

    @patch("builtins.open")
    @patch("json.load")
    def test_get_route(self, mock_json_load, mock_open):
        mock_json_load.return_value = {
            "routes": [{"routeId": "test_route", "steps": []}]
        }
        workflow = BaseWorkflow(self.config_path)
        route = workflow._get_route("test_route")
        assert route is not None
        assert route["routeId"] == "test_route"

    @patch("builtins.open")
    @patch("json.load")
    def test_execute_workflow(self, mock_json_load, mock_open):
        mock_json_load.return_value = {
            "routes": [{
                "routeId": "test_route",
                "steps": [{"step": 1, "skill": "test_skill"}]
            }]
        }
        workflow = BaseWorkflow(self.config_path)
        result = workflow.execute_workflow("test_route", {})
        assert result["status"] == "success"
