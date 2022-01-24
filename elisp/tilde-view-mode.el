(defun tilde-narrow-block ()
  (interactive)
  (save-excursion
    (forward-char)
    (let* ((bgn (re-search-backward "^/" nil t))
	   (end (re-search-forward "^/" nil t 2))
	   )
      (if end
	  (progn
	    (message (int-to-string end))
	    (narrow-to-region bgn (-  end 1))
	    )
	(narrow-to-region bgn (point-max))
	))))

(defun tilde-next-block ()
  (interactive)
  (forward-char)
  (re-search-forward "^/" nil 1)
  (backward-char))

(defun tilde-prev-block ()
  (interactive)
  (re-search-backward "^/" nil 1))

(defun tilde-next-block-with-narrow ()
  (interactive)
  (if (not (buffer-narrowed-p))
      (progn
	(tilde-next-block))
    (widen)
    (tilde-next-block)
    (tilde-narrow-block)
    ))

(defun tilde-prev-block-with-narrow ()
  (interactive)
  (if (not (buffer-narrowed-p))
      (progn
	(tilde-prev-block))
    (widen)
    (tilde-prev-block)
    (tilde-narrow-block)
    ))

(defun tilde-previous-line ()
  (interactive)
  (previous-line)
  (move-beginning-of-line 1))

(defun tilde-next-line ()
  (interactive)
  (next-line)
  (move-beginning-of-line 1))

(defun tilde-view-mode ()
  (interactive)
  (kill-all-local-variables)
  (setq major-mode 'tilde-view-mode
	mode-name "TILDE view mode")
  (outline-minor-mode)
  (setq outline-regexp " *")

  (setq tilde-view-mode-map (make-keymap))
  (suppress-keymap tilde-view-mode-map)
  (define-key tilde-view-mode-map (kbd "TAB") 'outline-toggle-children)
  (define-key tilde-view-mode-map "a" 'outline-show-all)
  (define-key tilde-view-mode-map "o" 'outline-hide-sublevels)

  (define-key tilde-view-mode-map "j" 'tilde-next-line)
  (define-key tilde-view-mode-map "k" 'tilde-previous-line)
  (define-key tilde-view-mode-map "n" 'tilde-next-block-with-narrow)
  (define-key tilde-view-mode-map "p" 'tilde-prev-block-with-narrow)

  (define-key tilde-view-mode-map "w" 'widen)
  (define-key tilde-view-mode-map (kbd "SPC") 'tilde-narrow-block)

  (use-local-map tilde-view-mode-map)
  )
