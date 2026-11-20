# ROSlean

ROS 2 nodes whose decision logic is written and proved in Lean. First target:
`twist_mux`. See [twist_mux.md](twist_mux.md) for scope and
[docs/PLAN.md](docs/PLAN.md) for the development plan.

## Quick start

Requires [elan](https://github.com/leanprover/elan) and
[pixi](https://pixi.sh). The ROS 2 Jazzy environment is installed into
`.pixi/` on first use.

```sh
pixi run noop      # build the Lean core and the ROS package, run the no-op node
```

In a second terminal:

```sh
pixi run bash -c 'source install/setup.bash && ros2 topic echo /roslean_noop/heartbeat'
```

Layout: `ROSleanCore/` is the Mathlib-free executable core linked into nodes,
`ROSlean/` holds proofs, `ros/` holds the ROS packages.


## GitHub configuration

To set up your new GitHub repository, follow these steps:

* Under your repository name, click **Settings**.
* In the **Actions** section of the sidebar, click "General".
* Check the box **Allow GitHub Actions to create and approve pull requests**.
* Click the **Pages** section of the settings sidebar.
* In the **Source** dropdown menu, select "GitHub Actions".

After following the steps above, you can remove this section from the README file.
