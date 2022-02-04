TILDE view mode
===

## About

Emacs major mode for viewing latency viewer stdout log.
You can see stdout log just like ncurse window.

## Usage

First, run tilde latency viewer with `--batch` mode.
Redirect stdout to file.

Open the log file, and `M-x tilde-view-mode`.
Of course, you need read tilde-view-mode.el in advance 
by putting load path, or `M-x eval-buffer` or other fancy way.

## Keymap

The latency viewer log file has space-index structure.
In the non batch mode, latency viewer shows lines between `/target/topic`.
Let's call these lines a BLOCK.

```
stamp ...
/target/topic
 /depth1/child/topic1
  /depth2/child/topic1
  /depth2/child/topic2
 /depth1/child/topic2
  /depth2/child/topic3
stamp ...
 (snip)
```

You can select "wide" and "narrow" view.

### wide view mode

In the wide view, we can see full text.
Additionally, we can fold sub trees by hitting "Tab" or "o".
Type "a" to show all lines.

### narrow view mode

Move cursor inside BLOCK, and hit "space" to narrow region.
Hit "n" or "p" to move BLOCKs.
Hit "w" to widen.

We can use the narrow view mode, just like latency viewer non-batch mode.
Morever, we can combine foldings.

