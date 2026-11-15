PYSRC = src/rcs_webui
TESTSRC = tests

pycheckformat:
	isort --check-only ${PYSRC} ${TESTSRC}
	black --check ${PYSRC} ${TESTSRC}

pyformat:
	isort ${PYSRC} ${TESTSRC}
	black ${PYSRC} ${TESTSRC}

ruff:
	ruff check ${PYSRC} ${TESTSRC}

pytest:
	python -m pytest ${TESTSRC}

.PHONY: pycheckformat pyformat ruff pytest
