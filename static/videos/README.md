# Video Assets - Git LFS Tracked

This directory contains video files for the Physical AI & Humanoid Robotics educational resource.

## Video Organization

```
videos/
├── chapter-openers/      # 30-second 4K chapter openers (13 videos)
│   ├── week-01-opener.mp4
│   ├── week-02-opener.mp4
│   └── ...
├── demos/                # Long-form demonstration videos
│   ├── week-03-gazebo-walking.mp4
│   ├── week-05-rl-timelapse.mp4
│   ├── week-06-vla-tasks.mp4
│   ├── week-09-jetson-benchmark.mp4
│   ├── week-11-first-hardware-walk.mp4
│   └── week-13-capstone-showcase.mp4
└── tutorials/            # Embedded tutorial videos
    └── ...
```

## Git LFS Configuration

All video files are tracked with Git LFS. See `.gitattributes` in the repository root:

```
*.mp4 filter=lfs diff=lfs merge=lfs -text
*.mov filter=lfs diff=lfs merge=lfs -text
*.webm filter=lfs diff=lfs merge=lfs -text
```

## Video Specifications

### Chapter Openers
- **Format**: MP4 (H.264)
- **Resolution**: 3840x2160 (4K)
- **Duration**: ~30 seconds
- **Bitrate**: 20 Mbps
- **File Size**: ~75 MB each

### Demo Videos
- **Format**: MP4 (H.264)
- **Resolution**: 1920x1080 (1080p) or 3840x2160 (4K)
- **Duration**: 3-10 minutes
- **Bitrate**: 10-20 Mbps
- **File Size**: ~200-600 MB each

## Production Guidelines

- Use Blender/Unity for cinematic renders
- Record actual robot footage at 60 fps
- Color grade for consistency
- Add subtitles/captions for accessibility
- Optimize file size with H.264 encoding

## Placeholder Status

Videos are currently placeholders and will be produced during content creation phase.

## License

All videos are licensed under the MIT License - see [LICENSE](../../LICENSE)
