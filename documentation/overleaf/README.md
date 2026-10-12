# Six Manipulator Project Report

This college-format report follows the supplied sample structure and documents the retained six-manipulator modelling and simulation work. It is not a claim of physical hardware validation.

## Overleaf
1. New Project > Upload Project: select this ZIP.
2. Set main.tex as the main document and pdfLaTeX as compiler.
3. Administrative details are populated in details.tex using the supplied registration number and faculty names. Confirm academic year (currently 2026–2027) and spellings before submission.
4. Recompile. Certificate signatures and institutional approval must be obtained separately.

## Local compilation
Run `latexmk -pdf main.tex`, or run `pdflatex main`, `bibtex main`, `pdflatex main`, `pdflatex main`.

## Contents and scope
- main.tex, details.tex, model_tables.tex: report source.
- references.bib and IEEEtran.bst: references and bibliography style.
- figures/: supplied or calculated figures used by the report.
- data/: retained feedback, workspace and trajectory exports and verification records.
- analysis/: supporting analysis scripts retained from the project paper; these are not required to compile the report.
- MANIFEST_SHA256.json: checksums of packaged source/evidence files.

The full ROS application is supplied separately; this ZIP is the report and evidence package. Serial tests cover J1/J2 excitation, while parallel tests cover the prescribed Delta and Stewart actuator trials. The UR5e adaptive workspace proposal is future work, not an implemented result. Signature spaces are intentionally unsigned. Do not treat the unsigned certificate as an issued institutional certificate.

## October 13 revision
The compiled report is 25 pages including front matter and references. The guide designation is Associate Professor. Extended model tables remain as supplementary source and are not included in the printed report. The cover uses the original embedded institutional logo with preserved aspect ratio and fixed placement.
