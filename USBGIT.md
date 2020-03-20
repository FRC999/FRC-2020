# How does Git on a USB Stick work, exactly?

While I'm out on my coronavacation, I figure I ought to write a bit
about something I know you have questions about.  How, exactly, does my
USB Git magic work?  Not just the commands to use it, but the conceptual
framework that it operates in.  (the raw commands are at the bottom).

## Abreviation notes

Repo = Repository = The full git repository = A complete reccord of
the history of a given project, as recorded by Git.

Local copy = the copy on your computer of the full repo

Remote = the repo on the other computer that is being synced to.

## The Actual Explaination

### Branches 101
In git, we put our work in branches, in order to keep track of what commit 
the branch is on
**PUT MORE EXPLAINATION HERE LATER: I HAVE TO GO FOR NOW**

## Commands

**NOTE TO SELF: Make sure all the paths are what I set up curly with**
The command is enclosed in the code block.  Be sure to read the paragraph
above the code block before running the command.  The paragraph below is
less important.

### Initialize the stick
On the USB stick, run the following to create and initalize the files.

`git clone --mirror github.com/FRC999/FRC-2020 FRC-2020-GitMirror.git`

This will likley take a while, because there is a bunch of latency between
git and the USB.  If you want, you could also pass your own local copy
to the command (without the `--mirror`), or drag the .git folder from your
copy to the stick (just the .git folder from inside your copy: rename it
to the full repo ID)

*999 note: I set up Curly and the other sticks to have the repo named
as above at the root directory.  Please replicate that for future USB
sticks*

### Initialize the computer
On your computer, open in VSCode the repo, and go to the terminal at
the bottom.  Alternativly, just get a terminal open somwhere in your
local copy of the repository.  Then, run

`git remote add USBStick G:\FRC-2020-GitMirror.git`

*999 note: I set up curly and the other sticks with a path similar
to the above.  Because git doesnt know if its Larry or Curly plugged in,
as long as the repo is in the same place on every stick, you can use 
the same USBStick remote for all of them.

### Push/pull to and from the stick

In a terminal that is in your copy (the 'terminal' tab on the bottom
of VS code works great):

`git pull USBStick/branchName`
`git push USBStick/branchName`
