#!/bin/sh
# Use: ./genplan.sh <plan_file> [<er_file>]
# Example: ./genplan.sh test.plan test.er

pnpgen_translator inline $1 $2

