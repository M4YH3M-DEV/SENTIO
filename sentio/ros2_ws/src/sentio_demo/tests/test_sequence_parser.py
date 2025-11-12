"""Tests for sequence parser."""

import pytest
import tempfile
from pathlib import Path
from sentio_demo.sequence_parser import SequenceParser


class TestSequenceParser:
    """Test suite for SequenceParser."""
    
    @pytest.fixture
    def parser(self):
        """Create parser instance."""
        return SequenceParser()
    
    @pytest.fixture
    def sample_yaml(self):
        """Create sample YAML sequence."""
        yaml_content = """
name: "Test Sequence"
description: "Test description"
steps:
  - action: "gesture"
    gesture: "greet"
    delay_ms: 0
    duration_ms: 1000
  - action: "tts"
    text: "Hello"
    delay_ms: 500
  - action: "wait"
    duration_ms: 1000
"""
        return yaml_content
    
    def test_load_sequence_valid(self, parser, sample_yaml):
        """Test loading valid sequence."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(sample_yaml)
            f.flush()
            
            sequence = parser.load_sequence(f.name)
            
            assert sequence is not None
            assert sequence['name'] == 'Test Sequence'
            assert len(sequence['steps']) == 3
            
            Path(f.name).unlink()
    
    def test_sequence_validation(self, parser):
        """Test sequence validation."""
        invalid_seq = {'invalid': 'structure'}
        assert parser._validate_sequence(invalid_seq) is False
        
        valid_seq = {'steps': [{'action': 'wait'}]}
        assert parser._validate_sequence(valid_seq) is True
    
    def test_parse_sequence_timing(self, parser, sample_yaml):
        """Test sequence timing calculation."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(sample_yaml)
            f.flush()
            
            sequence = parser.load_sequence(f.name)
            
            # Check timestamps are assigned
            steps = sequence['steps']
            assert steps[0]['timestamp_ms'] == 0  # First step at 0
            assert steps[1]['timestamp_ms'] == 1000  # Second at 1000ms
            
            Path(f.name).unlink()
    
    def test_variable_substitution(self, parser):
        """Test variable substitution."""
        sequence = {
            'name': 'Test',
            'steps': [
                {'action': 'tts', 'text': 'Hello {name}!'}
            ]
        }
        
        variables = {'name': 'Alice'}
        result = parser.substitute_variables(sequence, variables)
        
        assert result['steps'][0]['text'] == 'Hello Alice!'
    
    def test_list_sequences(self, parser, sample_yaml):
        """Test listing sequences."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            f.write(sample_yaml)
            f.flush()
            
            parser.load_sequence(f.name)
            
            sequences = parser.list_sequences()
            assert 'Test Sequence' in sequences
            
            Path(f.name).unlink()
