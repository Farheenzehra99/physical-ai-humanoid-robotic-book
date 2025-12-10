# URDF Designer - Usage Guide

## Example: Generate Humanoid URDF

```python
from skills.urdf_designer.src import URDFDesignerSkill

skill = URDFDesignerSkill()

result = skill.generate_humanoid_urdf(
    robot_name="humanoid_v1",
    dof_count=18,
    height_meters=1.2,
    output_format="xacro"
)

print(f"URDF created: {result['urdf_path']}")
```
