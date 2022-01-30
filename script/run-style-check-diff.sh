#!/bin/bash

set +e

# We need to add a new remote for the upstream master, since this script could
# be running in a personal fork of the repository which has out of date branches.
git remote add upstream https://github.com/RainerKuemmerle/g2o.git
git fetch upstream

# on github we are somehow not on the branch itself but in detached HEAD state.
# in order to run the check below we need to checkout our branch.
git checkout ${GITHUB_HEAD_REF}

# get the common ancestor from our branch to the base.
newest_common_ancestor_sha=$(git merge-base ${GITHUB_HEAD_REF} upstream/${GITHUB_BASE_REF})

# run the format-code - and count the lines
lines_diff_count=$(script/git-clang-format.py --diff ${newest_common_ancestor_sha} | wc -l)

# in case of no diff we return a one-liner
if [ ${lines_diff_count} -gt 1 ]; then
	echo "Format difference report:"
	script/git-clang-format.py --diff ${newest_common_ancestor_sha}
	exit 1
else
	exit 0
fi
