"""
Rule Evaluator

Evaluates policy rules against current system state.
"""

import logging
from typing import Dict, List, Any, Optional, Tuple
import operator


logger = logging.getLogger(__name__)


class RuleEvaluator:
    """
    Evaluates conditional rules to make decisions.
    
    Rules are dictionaries with:
    - 'condition': dict with field, operator, value
    - 'then': action/behavior to execute
    - 'priority': numeric priority (higher = more important)
    """
    
    # Supported operators
    OPERATORS = {
        'gt': operator.gt,
        'gte': operator.ge,
        'lt': operator.lt,
        'lte': operator.le,
        'eq': operator.eq,
        'ne': operator.ne,
        'in': lambda x, y: x in y,
        'not_in': lambda x, y: x not in y,
    }
    
    def __init__(self):
        """Initialize rule evaluator."""
        self.evaluated_rules: List[Dict] = []
    
    def evaluate_rule(
        self,
        rule: Dict,
        context: Dict
    ) -> Tuple[bool, Optional[str]]:
        """
        Evaluate a single rule against context.
        
        Args:
            rule: Rule dictionary with 'condition' and optional 'conditions'
            context: Current system state
        
        Returns:
            Tuple of (rule_matched, explanation)
        """
        try:
            # Handle simple condition
            if 'condition' in rule:
                matched, reason = self._evaluate_condition(
                    rule['condition'],
                    context
                )
                return matched, reason
            
            # Handle multiple conditions (AND logic)
            if 'conditions' in rule:
                all_matched = True
                reasons = []
                
                for condition in rule['conditions']:
                    matched, reason = self._evaluate_condition(
                        condition,
                        context
                    )
                    
                    if not matched:
                        all_matched = False
                    
                    if reason:
                        reasons.append(reason)
                
                explanation = ' AND '.join(reasons) if reasons else 'All conditions met'
                return all_matched, explanation
            
            logger.warning('Rule has no condition or conditions')
            return False, 'Rule missing conditions'
        
        except Exception as e:
            logger.error(f'Rule evaluation error: {str(e)}')
            return False, f'Error: {str(e)}'
    
    def _evaluate_condition(
        self,
        condition: Dict,
        context: Dict
    ) -> Tuple[bool, Optional[str]]:
        """
        Evaluate a single condition.
        
        Args:
            condition: Condition dict with 'field', 'operator', 'value'
            context: System state
        
        Returns:
            Tuple of (condition_met, explanation)
        """
        try:
            field = condition.get('field')
            operator_name = condition.get('operator')
            value = condition.get('value')
            
            if not field or not operator_name or value is None:
                logger.warning('Condition missing required fields')
                return False, None
            
            # Get field value from context
            context_value = self._get_nested_value(context, field)
            
            if context_value is None:
                return False, f'Field not found: {field}'
            
            # Get operator function
            op_func = self.OPERATORS.get(operator_name)
            if not op_func:
                logger.warning(f'Unknown operator: {operator_name}')
                return False, None
            
            # Evaluate
            result = op_func(context_value, value)
            
            # Create explanation
            explanation = f'{field} {operator_name} {value} ({context_value})'
            
            return result, explanation
        
        except Exception as e:
            logger.error(f'Condition evaluation error: {str(e)}')
            return False, None
    
    def _get_nested_value(self, obj: Dict, path: str) -> Optional[Any]:
        """
        Get nested value from dict using dot notation.
        
        Args:
            obj: Dictionary to query
            path: Dot-separated path (e.g., 'affect.valence')
        
        Returns:
            Value or None if not found
        """
        keys = path.split('.')
        current = obj
        
        for key in keys:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None
        
        return current
    
    def evaluate_rules(
        self,
        rules: List[Dict],
        context: Dict,
        match_first: bool = False
    ) -> List[Dict]:
        """
        Evaluate multiple rules in priority order.
        
        Args:
            rules: List of rules to evaluate
            context: System state
            match_first: Return after first match if True
        
        Returns:
            List of matched rules with metadata
        """
        matched_rules = []
        
        # Sort by priority (descending)
        sorted_rules = sorted(
            rules,
            key=lambda r: r.get('priority', 0),
            reverse=True
        )
        
        for rule in sorted_rules:
            result, explanation = self.evaluate_rule(rule, context)
            
            if result:
                matched_rules.append({
                    'rule': rule,
                    'matched': True,
                    'explanation': explanation,
                    'priority': rule.get('priority', 0)
                })
                
                if match_first:
                    break
        
        self.evaluated_rules = matched_rules
        return matched_rules
    
    def get_evaluation_summary(self) -> Dict:
        """Get summary of last evaluation."""
        return {
            'rules_evaluated': len(self.evaluated_rules),
            'matched_count': sum(1 for r in self.evaluated_rules if r['matched']),
            'rules': self.evaluated_rules
        }
