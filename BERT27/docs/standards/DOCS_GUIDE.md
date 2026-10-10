# Docs Guide


## Types of documents

**These are some ideas for what I beleive we will need (some could be better named)**

1. **Reference Documents** includes templates and anything that needs to be references such as tables, datasheets, etc. 
2. **Explanation Documents** Where we would store rough drafts of documentation for each subsection of the codebase. For example we would list functions and their parameters, what each file is for, etc.
3. **Offical Documents** Finalized versions of the explanation documents.
4. **Standards** Standards for documentation, formatting code, etc. s


Link documents to each other where needed. also make sure to follow the folder structure in [FILE_STRUCTURE_GUIDE.md](FILE_STRUCTURE_GUIDE.md).

## Every board needs a README

The README sits in the board's folder (`firmware/vcu/README.md`) and covers:

1. what the board does
2. how to build and flash it
3. pin table
4. state machine diagram
5. fault table: name, trigger, action, how it clears
6. CAN messages it sends and receives
7. log files it writes
8. values you can tune at the track
9. known issues (links to GitHub issues)

For the layout, check `../templates/BOARD_README_TEMPLATE.md`.

## Official documents

`docs/official/` holds explanation docs that are fully finished.

**NEEDS TO BE UPDATED ONCE FORMAT OF DOCS ARE FINALIZED**

## CAN messages

**TODO**


## Formal team documents

Official documents should be designed with the final Word documentation from SharePoint (i.e. the EDAP Word doc) if we decide to document the codebase the same way as other things are being documented on the sharepoint.