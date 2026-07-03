---
id: "make-a-sampling-version-with-the-esp32-audio-board-2026-06-25"
status: "in-progress"
priority: "high"
assignee: null
epic: null
dueDate: null
created: "2026-06-25T11:15:09.281Z"
modified: "2026-06-25T11:23:15.846Z"
completedAt: null
labels: []
order: "a1"
---
# make a sampling variant with the esp32 audio kit v2.2 A247  and mpr121s

- lets start by making two modes working

  - press key 1 and key3-key6 to record a sample on them that gets trimmed the starting silence off and then gets played start to end when you press key 3-6 again
  - press key 2 and key3-key6 to record a sample on them that gets trimmed the starting silence off and then gets played as an endless granular synthesis tone (with the start of the sample at the start, and the end of the sample train off when you let go)when you press key 3-6 again

- when that is working we can connect it to mpr121s again for the cap sensing ( I will decide later how many pads, can you make a suggestion for how many samples (of how long) I could hold?

- then also add the screen that the 12pad-screen version has (can you tell me first if there is enough IO on the board