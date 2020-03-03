# FRC-2020

Code for the 2020 FIRST Robotics Competition!

## Git explained

The "Pro Git" book is an excellent piece of free documentation for Git.  I highly recommend you read it, because if you do, you will have a comprehensive knowledge of git.  

The Pro Git book can be found [here.](https://git-scm.com/book/en/v2)
Additionally, please check out this article on writing commits: [How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/).  Although having an expansive body as described in that post is less vital for us, *please* follow the rules for the subject line.  That means never titling them "Commit": **ever.**

### Git for the busy Gits who won't read the book

Git has four main types of objects that I am going to talk about: trees, commits, branches, and blobs.  A blob is a file: git keeps track of them (and most other objects) by making a hash of them, which is a way to assign a unique identifier to a file based solely on its content.  Remember that word, hash: git hashes a lot of things.

So, a tree is the directory structure: think of it as a tree, whose base is the root direction (get it, root?), and the branches are subdirectories, with the leaves being blobs (files).  Now, remember how I said that git tracks objects by their hashes?  Well, what that means is that if a file is unchanged, git will not store another copy: rather, git will just reference the previous copy.

(It's worth noting that just doing that is inefficient: after all, changing one character in a large file will drastically change the hash, and so the large file would be stored all over again.  Git has a quite clever way of packing those together: it's called packing, and it's what is going on whenever git talks about deltas)

Now, the next up in the hierarchy of git objects is the commit.  A commit contains a pointer to a tree, but it also states what commit it is based on, who committed it and when, and a custom message that they would have added.  Commits are what you work with a lot in git: while trees and blobs are good to know about, because they are how git works, it's important to understand commits first.  However, the author is getting tired, and so they won't go further.

TODO: expand on the above, and explain branches and such

### Git Branch Structure

This git repository is designed to allow us to track the various states of confidence
that we have in code with more fidelity than simply "completed" and "in progress".
Last year, we discovered that the code in our master branch was nonfunctional: we thus
did all of our development in a second branch, HartfordCode, leaving master as a
useless relic.

Thus, I am introducing a new system of branching, with three main branches, and all
development taking place in sub-branches.  We attempted to do development in sub-branches
last year, however, this year, we will do this slightly differently.  The main branches are
as follows: (totally not based on the three-releases model of Debian)

#### 'Stable' branch

The *Stable* branch is the most stable of the branches: it is only for code that has been
both thoroughly tested and documented, and can be considered "competition ready".  **It
must not be directly committed to**, except in cases of vital bugfixes: instead, bring
in changes from 'master'.  *Stable* is **intentionally** distinct from master; Code
in *master* should be tested and stable, but code in *stable* should be extremely tested
and very stable.  Additionally, release tags should be made on stable branch heads.

#### 'Master' branch

*Master* branch is just a typical master branch (it happens to be the equivalent of Debian
testing).  Thus, it is code that can be loaded on to the robot before a competition and
expected to run.  Commits directly to *master* are permitted, but only if they are tested
(eg, bugfixes made during a testing session), and are not recommended (you should make the
fixes before code enters master).  The *master* branch can only be set to commits that are
known to work: period.  If master is being updated to a less-well-tested version, it is
recommended to update stable to point to where master was.  If the code currently in stable
is still desired, a release tag should be used.

#### 'Sid' branch

*Sid* is based on the debian branch of the same name.  It is where code suitable  for being
tested is put.  Code in *sid* may be buggy: it should not be used for a competition.
*Sid* may be directly committed to as part of a bugfixes, but commits to *sid* should
try to leave *sid* in a state that is somewhat suitable for a competition bot.  Thus,
new features should be developed in separate branches, and then transferred over to *sid*.

### Git submodules

We use(d) a git submodule to bring in Witchcraft, our reusable set of java classes.  I'll
write more on that later: currently, I need to wrap up kickoff day preparations.
