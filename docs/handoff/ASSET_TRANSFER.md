# Atlas Map Asset Transfer Guide

The Atlas Autoware full-stack demonstration requires three external map assets
that are not bundled in the Git repository.

## 1. Required Files

| File | Description | SHA-256 |
|---|---|---|
| `finalMap_repaired.osm` | Lanelet2 vector map of the campus. | `1e9360ea62b709271007d27672120b3d0040ebabc6667d646de68734b1702fee` |
| `map_projector_info.yaml` | Georeferencing projection parameters. | `614af74d6aaefe052c435338b585797a451e7922efc290329374c2abd2b8992d` |
| `pointcloud_map_georeferenced.pcd` | Reference point cloud map for visualization. | `6121b6bf8df4324ab42286822f0e6f00c67ecc10daecf84d180129a6069ef9a3` |

## 2. Verify the source files

On the reference VM, verify the immutable transfer inputs before copying:

```bash
cd /home/marian/roam/atlas_autoware
(cd /home/marian/Downloads && sha256sum -c \
  /home/marian/roam/atlas_autoware/tools/native_ubuntu/map_assets.sha256)
```

All three files must report `OK`. Stop if any hash differs.

## 3. Transfer to Vyapak's laptop

On the destination laptop, create the documented portable location:

```bash
mkdir -p "$HOME/atlas_assets"
```

Use one approved transport. For SSH/`rsync`, run this on the destination and
replace `<reference-vm-host>` with the reference VM's approved SSH host or IP:

```bash
rsync -av --progress \
  marian@<reference-vm-host>:/home/marian/Downloads/finalMap_repaired.osm \
  marian@<reference-vm-host>:/home/marian/Downloads/map_projector_info.yaml \
  marian@<reference-vm-host>:/home/marian/Downloads/pointcloud_map_georeferenced.pcd \
  "$HOME/atlas_assets/"
```

If only `scp` is available:

```bash
scp \
  marian@<reference-vm-host>:/home/marian/Downloads/finalMap_repaired.osm \
  marian@<reference-vm-host>:/home/marian/Downloads/map_projector_info.yaml \
  marian@<reference-vm-host>:/home/marian/Downloads/pointcloud_map_georeferenced.pcd \
  "$HOME/atlas_assets/"
```

Do not rename, edit, convert, or line-ending-normalize the files.

## 4. Verify the destination files

From the destination clone:

```bash
cd "$HOME/atlas_autoware"
export ATLAS_MAP_DIR="$HOME/atlas_assets"
(cd "$ATLAS_MAP_DIR" && sha256sum -c \
  "$HOME/atlas_autoware/tools/native_ubuntu/map_assets.sha256")
```

All files must report `OK`. The repository checksum manifest is authoritative;
do not continue to build or launch after a mismatch.

Export `ATLAS_MAP_DIR` in each Atlas shell:

```bash
export ATLAS_MAP_DIR="$HOME/atlas_assets"
```

## 5. Usage in launch

The launch files use `ATLAS_MAP_DIR` to locate these files. The portable fallback
is `~/atlas_assets`, but exporting the variable explicitly makes the selected
asset set auditable.

If you need to override individual paths at launch time:
```bash
ros2 launch atlas_full_stack_demo full_stack.launch.py \
  obstacle:=false request_engagement:=false \
  osm_path:=/custom/path/map.osm \
  pcd_path:=/custom/path/map.pcd \
  projector_path:=/custom/path/projector.yaml
```
