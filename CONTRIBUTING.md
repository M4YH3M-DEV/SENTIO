# Contributing to SENTIO

Thank you for your interest in contributing to the SENTIO robot project! This guide will help you understand our development process and how to submit contributions.

## Table of Contents

1. [Code of Conduct](#code-of-conduct)
2. [Getting Started](#getting-started)
3. [Development Workflow](#development-workflow)
4. [Coding Standards](#coding-standards)
5. [Testing](#testing)
6. [Commit Guidelines](#commit-guidelines)
7. [Pull Request Process](#pull-request-process)
8. [Release Process](#release-process)

## Code of Conduct

We are committed to providing a welcoming and inclusive environment. All contributors must:

- Be respectful of diverse viewpoints
- Give credit where credit is due
- Focus on constructive feedback
- Report inappropriate behavior to dev@devsora.tech

## Getting Started

### Prerequisites

- Ubuntu 24.04 or equivalent
- Python 3.12
- Node.js 20+
- ROS 2 Rolling
- Git

### Initial Setup

Clone repository
git clone https://github.com/devsora/sentio.git
cd sentio

Install development dependencies
python -m pip install -r requirements-dev.txt
cd ui && npm ci && cd ..

Run local CI
./ci/test-runner.sh

text

## Development Workflow

### 1. Create Feature Branch

Always branch from develop
git checkout develop
git pull origin develop

Create feature branch with descriptive name
git checkout -b feature/your-feature-name

or
git checkout -b fix/your-bug-fix

text

### 2. Make Changes

- Follow coding standards (see below)
- Add tests for new functionality
- Update documentation
- Run local tests frequently

Run linting
black sentio_ros2_ws/ ui/ sentio/tools/
isort sentio_ros2_ws/ ui/ sentio/tools/

Run tests
pytest sentio_ros2_ws/src/*/tests/
cd ui && npm test && cd ..

Build locally
cd sentio_ros2_ws
colcon build --symlink-install

text

### 3. Commit Changes

Follow [conventional commits](#commit-guidelines) format.

### 4. Push & Create PR

git push origin feature/your-feature-name

text

Then create PR on GitHub with detailed description.

## Coding Standards

### Python

- **Style**: Black (100-char line length)
- **Linter**: Flake8, isort
- **Type Hints**: Required for public APIs
- **Docstrings**: Google-style for classes and functions

def analyze_emotion(valence: float, arousal: float) -> str:
"""
Analyze emotion based on valence and arousal scores.

text
Args:
    valence: Valence score (-1.0 to +1.0)
    arousal: Arousal score (0.0 to 1.0)

Returns:
    Emotion label (e.g., 'happy', 'sad')
"""
if valence > 0.5 and arousal > 0.5:
    return 'happy'
# ... etc
text

### TypeScript/React

- **Style**: Prettier (100-char line length)
- **Linter**: ESLint + TypeScript
- **Components**: Functional components with hooks
- **Props**: Use interfaces, avoid any

interface EmotionCardProps {
emotion: string;
confidence: number;
onDismiss?: () => void;
}

export function EmotionCard({
emotion,
confidence,
onDismiss,
}: EmotionCardProps) {
// Component implementation
}

text

### C++/Firmware

- **Style**: LLVM style
- **Naming**: snake_case for functions, camelCase for variables
- **Comments**: Document public APIs and complex logic

## Testing

### Python Tests

Run all tests
pytest sentio_ros2_ws/src -v --cov

Run specific test
pytest sentio_ros2_ws/src/sentio_perception/tests/test_emotion.py -v

Run with coverage report
pytest --cov=sentio_ros2_ws --cov-report=html

text

### UI Tests

cd ui

Run unit tests
npm test

Generate coverage report
npm test -- --coverage

Linting
npm run lint
npm run format -- --check

text

### ROS 2 Integration Tests

cd sentio_ros2_ws
source /opt/ros/rolling/setup.bash

Build with testing enabled
colcon build --packages-select sentio_core --cmake-args -DENABLE_TESTING=ON

Run tests
colcon test --packages-select sentio_core

text

### New Feature Checklist

- [ ] Unit tests written and passing
- [ ] Integration tests written and passing
- [ ] Code coverage > 80% for new code
- [ ] Documentation updated
- [ ] No breaking changes, or deprecation warning added
- [ ] Type hints added (Python) or TypeScript types (JS)

## Commit Guidelines

Follow [Conventional Commits](https://www.conventionalcommits.org/):

type(scope): subject

body (optional)

footer (optional)

text

### Types

- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Formatting, missing semicolons, etc.
- `refactor`: Code refactoring without behavior change
- `perf`: Performance improvement
- `test`: Adding/updating tests
- `chore`: Build, CI, dependency updates

### Examples

feat(policy): add context-aware behavior mapping

Implement contextual modifiers for gesture selection
based on group size and proximity.

Closes #42

text
undefined
fix(motion): servo velocity clamping calculation

Ensure servo commands respect maximum velocity limits.
Fixes race condition in multi-servo scenarios.

text
undefined
docs(readme): add hardware setup instructions

text

## Pull Request Process

### 1. Before Submitting

- [ ] Branch is up-to-date with develop
- [ ] All tests pass locally
- [ ] Code formatted and linted
- [ ] Commit messages follow guidelines
- [ ] No merge conflicts

### 2. Create PR

Use PR template (auto-populated):

Description
Brief description of changes.

Type of Change
 Bug fix

 New feature

 Breaking change

 Documentation

Testing
Describe test coverage.

Checklist
 Tests pass

 Documentation updated

 No new warnings

text

### 3. Reviewer Responsibilities

Reviewers will:
- Check code quality and standards
- Verify test coverage
- Request changes if needed
- Approve when satisfied

### 4. Merge

PR will be merged after:
- All CI checks pass
- At least one approval
- No unresolved comments

## Release Process

### Version Numbering

Follows [Semantic Versioning](https://semver.org/):

- MAJOR: Incompatible API changes
- MINOR: New functionality (backward compatible)
- PATCH: Bug fixes

### Release Steps

1. Update version in code
2. Create annotated tag: `git tag -a v1.0.0 -m "Release v1.0.0"`
3. Push tag: `git push origin v1.0.0`
4. GitHub Actions auto-creates release with artifacts
5. Announce in documentation

Example
git tag -a v1.2.0 -m "Release v1.2.0 - New emotion recognition model"
git push origin v1.2.0

text

## Questions?

- Check [DEVELOPER_SETUP.md](DEVELOPER_SETUP.md) for local setup
- Review [CI_README.md](CI_README.md) for CI/CD details
- Email: dev@devsora.tech

---

**Last Updated**: 2025-11-13
**Maintained By**: DevSora Deep-Tech Research