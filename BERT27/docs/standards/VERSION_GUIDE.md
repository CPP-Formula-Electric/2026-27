# Version Guide

We are following Semantic Versioning 2.0.0 so check the site as it goes more in depth.
https://semver.org/spec/v2.0.0.html 

## How the numbering system works

Each board has its own version number with three parts: **MAJOR.MINOR.PATCH**.

| Part | Change it when |
|---|---|
| MAJOR (<code><span style="color:#79c0ff">2</span>.0.0</code>) | changes which would cause previous versions to stop working |
| MINOR (<code>1.<span style="color:#79c0ff">5</span>.0</code>) | the car behaves in a new way |
| PATCH (<code>1.4.<span style="color:#79c0ff">1</span></code>) | you fixed a bug and changed nothing else |


## Test day versions

Code that is ready for the car after being reviewed will get the -rc.1, -rc.2, etc. tags onto the version numbers. For example `1.5.0-rc.1`. Once it is proven to work properly on the car, it becomes `1.5.0`.

## Tags

You set the version by putting a git tag on a commit:

```
git tag -a vcu-v1.5.0 -m "VCU 1.5.0: new regen limit"
git push origin vcu-v1.5.0
```

Tag format: `<board>-v<version>`, such as `vcu-v1.5.0` or `bms-v2.0.1-rc.1`.

Never move or delete a tag once you've pushed it. If a release is broken, make a new PATCH version.