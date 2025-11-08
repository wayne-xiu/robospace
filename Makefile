# Robospace Makefile
# Provides convenient targets for building, testing, and development

.PHONY: help configure-debug configure-release configure-dev build build-cpp \
        build-python test test-cpp test-python clean clean-all install \
        install-cpp install-python format lint docs

# Default target
help:
	@echo "Robospace Development Targets"
	@echo "=============================="
	@echo ""
	@echo "Configuration:"
	@echo "  configure-debug   - Configure CMake for debug build (with tests)"
	@echo "  configure-release - Configure CMake for release build (no tests)"
	@echo "  configure-dev     - Configure for development (Release + tests + compile_commands)"
	@echo ""
	@echo "Building:"
	@echo "  build            - Build C++ library and Python bindings"
	@echo "  build-cpp        - Build C++ library only"
	@echo "  build-python     - Build Python bindings only"
	@echo ""
	@echo "Testing:"
	@echo "  test             - Run all tests (C++ and Python)"
	@echo "  test-cpp         - Run C++ tests only"
	@echo "  test-python      - Run Python tests only (pytest)"
	@echo ""
	@echo "Installation:"
	@echo "  install          - Install C++ library and Python package"
	@echo "  install-cpp      - Install C++ library system-wide"
	@echo "  install-python   - Install Python package in editable mode"
	@echo ""
	@echo "Cleaning:"
	@echo "  clean            - Remove build directory"
	@echo "  clean-all        - Remove all build artifacts (C++ and Python)"
	@echo ""
	@echo "Development (future):"
	@echo "  format           - Format C++ code with clang-format"
	@echo "  lint             - Lint C++ and Python code"
	@echo "  docs             - Generate documentation"
	@echo ""

# Build directory
BUILD_DIR := build
BUILD_TYPE ?= Release
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

#
# Configuration
#

configure-debug:
	@echo "Configuring CMake (Debug with tests)..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. \
		-DCMAKE_BUILD_TYPE=Debug \
		-DBUILD_TESTS=ON \
		-DBUILD_PYTHON=OFF \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@echo "✓ Configuration complete (Debug)"

configure-release:
	@echo "Configuring CMake (Release for deployment)..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTS=OFF \
		-DBUILD_PYTHON=OFF \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=OFF
	@echo "✓ Configuration complete (Release)"

configure-dev:
	@echo "Configuring CMake (Development mode)..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. \
		-DCMAKE_BUILD_TYPE=Release \
		-DBUILD_TESTS=ON \
		-DBUILD_PYTHON=OFF \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON
	@echo "✓ Configuration complete (Dev mode)"

# Auto-configure if build directory doesn't exist (use dev mode for convenience)
$(BUILD_DIR)/Makefile:
	@$(MAKE) configure-dev

#
# Building
#

build: $(BUILD_DIR)/Makefile
	@echo "Building robospace (C++ and Python)..."
	cd $(BUILD_DIR) && $(MAKE) -j$(NPROC)
	@echo "✓ Build complete"

build-cpp: $(BUILD_DIR)/Makefile
	@echo "Building C++ library..."
	cd $(BUILD_DIR) && $(MAKE) robospace-math -j$(NPROC)
	@echo "✓ C++ build complete"

build-python: build-cpp
	@echo "Building Python bindings..."
	@if [ -d python/bindings ]; then \
		pip install -e . --no-build-isolation; \
	else \
		echo "⚠ Python bindings not yet implemented"; \
	fi
	@echo "✓ Python build complete"

#
# Testing
#

test: test-cpp test-python

test-cpp: build
	@echo "Running C++ tests..."
	cd $(BUILD_DIR) && ctest --output-on-failure
	@echo "✓ C++ tests passed"

test-python:
	@echo "Running Python tests..."
	@if [ -d tests/python ] && [ -f tests/python/test_*.py ]; then \
		pytest tests/python -v; \
	else \
		echo "⚠ Python tests not yet implemented"; \
	fi
	@echo "✓ Python tests complete"

#
# Installation
#

install: install-cpp install-python

install-cpp: build-cpp
	@echo "Installing C++ library..."
	cd $(BUILD_DIR) && sudo $(MAKE) install
	@echo "✓ C++ library installed"

install-python:
	@echo "Installing Python package (editable mode)..."
	pip install -e .
	@echo "✓ Python package installed"

#
# Cleaning
#

clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
	@echo "✓ Clean complete"

clean-all: clean
	@echo "Cleaning all build artifacts..."
	@rm -rf dist *.egg-info
	@find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@find . -type f -name "*.pyo" -delete 2>/dev/null || true
	@echo "✓ Deep clean complete"

#
# Development tools (future implementation)
#

format:
	@echo "⚠ Code formatting not yet configured"
	@echo "TODO: Run clang-format on C++ files and black on Python files"

lint:
	@echo "⚠ Linting not yet configured"
	@echo "TODO: Run clang-tidy, cppcheck, ruff/pylint"

docs:
	@echo "⚠ Documentation generation not yet configured"
	@echo "TODO: Generate Doxygen (C++) and Sphinx (Python) docs"
