---
hide:
  - footer
---
# Generating Documentation

This section is a reminder on how to update this documentation. Start by installing the required python modules using the `requirements.txt` file located in the docs folder:

```bash
    python3 -m pip install -r requirements.txt
```

Then to build the documentation for a specific version use:

```bash
    mike deploy <version>
```
!!! note

    The `--push` tag is used to deploy the documentation version to *Github Pages* and the `--update-alias` tag is used to update the version alias if it has changed.

To view the documentation locally use:

```bash
    mike serve
```

To change a version alias use:

```bash
    mike alias <version> <alias>
```