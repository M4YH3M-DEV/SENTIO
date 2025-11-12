"""Tests for rule evaluator."""

import pytest
from sentio_policy.rule_evaluator import RuleEvaluator


class TestRuleEvaluator:
    """Test suite for RuleEvaluator."""
    
    @pytest.fixture
    def evaluator(self):
        """Create evaluator instance."""
        return RuleEvaluator()
    
    @pytest.fixture
    def sample_context(self):
        """Sample evaluation context."""
        return {
            'affect': {
                'valence': 0.7,
                'arousal': 0.5,
                'label': 1
            },
            'context': {
                'proximity_m': 0.8,
                'group_count': 1
            }
        }
    
    def test_simple_condition_gt(self, evaluator, sample_context):
        """Test greater than operator."""
        rule = {
            'condition': {
                'field': 'affect.valence',
                'operator': 'gt',
                'value': 0.5
            }
        }
        
        result, _ = evaluator.evaluate_rule(rule, sample_context)
        assert result is True
    
    def test_simple_condition_lt(self, evaluator, sample_context):
        """Test less than operator."""
        rule = {
            'condition': {
                'field': 'affect.valence',
                'operator': 'lt',
                'value': 0.5
            }
        }
        
        result, _ = evaluator.evaluate_rule(rule, sample_context)
        assert result is False
    
    def test_multiple_conditions_and(self, evaluator, sample_context):
        """Test AND logic for multiple conditions."""
        rule = {
            'conditions': [
                {
                    'field': 'affect.valence',
                    'operator': 'gt',
                    'value': 0.5
                },
                {
                    'field': 'context.proximity_m',
                    'operator': 'lt',
                    'value': 1.0
                }
            ]
        }
        
        result, _ = evaluator.evaluate_rule(rule, sample_context)
        assert result is True
    
    def test_nested_field_access(self, evaluator):
        """Test accessing nested fields."""
        context = {
            'data': {
                'nested': {
                    'value': 42
                }
            }
        }
        
        value = evaluator._get_nested_value(context, 'data.nested.value')
        assert value == 42
    
    def test_missing_field(self, evaluator, sample_context):
        """Test handling of missing fields."""
        rule = {
            'condition': {
                'field': 'nonexistent.field',
                'operator': 'gt',
                'value': 0.5
            }
        }
        
        result, explanation = evaluator.evaluate_rule(rule, sample_context)
        assert result is False
        assert 'not found' in explanation.lower()


class TestRuleEvaluation:
    """Test suite for multi-rule evaluation."""
    
    def test_rule_priority_sorting(self):
        """Test rules are evaluated by priority."""
        evaluator = RuleEvaluator()
        
        rules = [
            {'name': 'low', 'priority': 10, 'condition': {'field': 'a', 'operator': 'eq', 'value': 1}},
            {'name': 'high', 'priority': 100, 'condition': {'field': 'a', 'operator': 'eq', 'value': 1}},
            {'name': 'mid', 'priority': 50, 'condition': {'field': 'a', 'operator': 'eq', 'value': 1}},
        ]
        
        context = {'a': 1}
        results = evaluator.evaluate_rules(rules, context)
        
        # Should be ordered by priority
        names = [r['rule']['name'] for r in results]
        assert names.index('high') < names.index('mid')
        assert names.index('mid') < names.index('low')
    
    def test_first_match_only(self):
        """Test match_first returns after first match."""
        evaluator = RuleEvaluator()
        
        rules = [
            {'priority': 10, 'condition': {'field': 'a', 'operator': 'eq', 'value': 1}},
            {'priority': 20, 'condition': {'field': 'a', 'operator': 'eq', 'value': 1}},
        ]
        
        context = {'a': 1}
        results = evaluator.evaluate_rules(rules, context, match_first=True)
        
        assert len(results) == 1
