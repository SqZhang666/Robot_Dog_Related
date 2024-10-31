#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo -e "Dir is ${DIR}"
export PYTHONPATH=${DIR}/../common/lcm_type/lcm/:${DIR}:${DIR}/lcm-log2smat/python:${PYTHONPATH}
echo -e "Python path is ${PYTHONPATH}"

if [[ $(lsb_release -rs) == "20.04" ]]; then
  echo "Use Python3 on Ubuntu 20.04"
  exec /usr/bin/python3 -m lcmlog2smat.log_to_smat $1 -o $2
else
  echo "Use Python on systems other than Ubuntu 20.04"
  exec /usr/bin/python -m lcmlog2smat.log_to_smat $1 -o $2
fi
