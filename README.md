# FRC-2020
Code for the 2020 FIRST Robotics Competition!

TODO: Setup engineering log + scripts to generate beauty

# Git explained

## Git Branch Structure
This git repository is designed to allow us to track the various states of confidence
that we have in code with more fidelity than simply "completed" and "in progress".
Last year, we discovered that the code in our master branch was nonfunctional: we thus
did all of our development in a seccond branch, HartfordCode, leaving master as a
useless relic.

Thus, I am introducing a new system of branching, with three main branches, and all
development taking place in sub-branches.  We attempted to do development in sub-branches
last year, however, this year, we will do this slightly differently.  The branches are
as follows: (totally not based on the three-releases model of Debian)

### 'Stable' branch
The *Stable* branch is the most stable of the branches: it is only for code that has been
both thoughly tested and documented, and can be considered "competition ready".  **It 
must not be directly committed to**, except in cases of vital bugfixes: instead, bring
in changes from 'master'.  *Stable* is **intentionally** distinct from master; Code 
in *master* should be tested and stable, but code in *stable* should be extreamly tested
and very stable.  Additionally, release tags should be made on stable branch heads.

###  'Master' branch
*Master* branch is just a typical master branch (it happens to be the equivalent of Debin
testing).  Thus, it is code that can be loaded on to the robot before a competition and
expected to run.  Commits directly to *master* are permitted, but only if they are tested
(eg, bugfixes made during a testing session), and are not reccomended (you should make the
fixes before code enters master).  The *master* branch can only be set to commits that are 
known to work: period.  If master is being updated to a less-well-tested version, it is
reccomended to update stable to point to where master was.  If the code currently in stable 
is still desired, a release tag should be used.

### 'Sid' branch
*Sid* is based on the debian branch of the same name.  It is where code suitable  for being
tested is put.  Code in *sid* may be buggy: it should not be used for a competition.
*Sid* may be directly commited to as part of a bugfix, but commits to *sid* should
try to leave *sid* in a state that is more suitable for a competition bot.  Thus,
new features should be developed in seperate branches, and then transfered over to *sid*.

## Git submodules
We use a git submodule to bring in Witchcraft, our reusable set of java classes.  I'll
write more on that later: currently, I need to wrap up kickoff day preperations.
