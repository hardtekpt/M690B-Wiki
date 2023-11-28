---
hide:
  - footer
---
# Generating Documentation

This section is a reminder on how to update this documentation. On top of **MKdocs**, [**Mike**](https://github.com/jimporter/mike) is being used as a version system. It is noted that the version `3.8` of **Python** was used.

## Prerequesites

Start by installing the required python modules using the `requirements.txt` file located in the docs folder:

```bash
    python3 -m pip install -r requirements.txt
```

## Compiling and Deploying

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

## Github Actions

In order to aumotate the deployment of the documentation a Gihtub Action has been used. Every time changes are pushed to main branch and action is triggered to build and deploy the documentation. To control which version of the documentation will be built, a VERSION file on the root of the repository contains the current version.
