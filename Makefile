# This repository is a component of ImplusOS (github.com/ImplusOS).
# Standalone use requires the full ImplusOS tree (siblings: I_libc/, Library/,
# Vendor/, Build/) checked out at the same level as this repository -- see
# README.md. All actual build rules live in Source/.

.PHONY: all clean

all clean:
	@$(MAKE) -C Source $@
