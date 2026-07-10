"""Enable `python -m galdr_scope` as an alias for the `scope-capture` entry point."""

from galdr_scope.cli import main

if __name__ == "__main__":
  raise SystemExit(main())
