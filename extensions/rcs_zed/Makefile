PYSRC = src/rcs_zed
TESTSRC = tests
TESTFILE = tests/test_zed_extension.py
PYTHONPATH_LOCAL = ../../python:src:../rcs_realsense/src

pycheckformat:
	isort --check-only ${PYSRC} ${TESTSRC}
	black --check ${PYSRC} ${TESTSRC}

pyformat:
	isort ${PYSRC} ${TESTSRC}
	black ${PYSRC} ${TESTSRC}

ruff:
	ruff check ${PYSRC} ${TESTSRC}

pytest:
	PYTHONPATH=${PYTHONPATH_LOCAL} python -m pytest ${TESTFILE}

test-compile:
	python -m compileall ${PYSRC} ${TESTFILE}

.PHONY: pycheckformat pyformat ruff pytest test-compile
