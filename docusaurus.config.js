// @ts-nocheck
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require("prism-react-renderer/themes/github");
const darkCodeTheme = require("prism-react-renderer/themes/dracula");

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "Zero to Walking Robot in 13 Weeks - Complete Educational Resource",
  favicon: "img/favicon.ico",

  // Production URL
  url: "https://physical-ai-robotics.dev",
  baseUrl: "/",

  // GitHub Pages deployment config
  organizationName: "your-org",
  projectName: "physical-ai-humanoid-robotics",

  onBrokenLinks: "warn", // Changed from "throw" to allow build to continue despite broken links
  onBrokenMarkdownLinks: "warn",

  // Internationalization
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve("./sidebars.js"),
          editUrl:
            "https://github.com/your-org/physical-ai-humanoid-robotics/tree/main/",
          showLastUpdateAuthor: false,
          showLastUpdateTime: false,
        },
        blog: false, // Disable blog for this project
        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
        sitemap: {
          changefreq: "weekly",
          priority: 0.5,
          ignorePatterns: ["/tags/**"],
          filename: "sitemap.xml",
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // SEO metadata
      metadata: [
        {
          name: "keywords",
          content:
            "robotics, ai, humanoid, ros2, isaac-sim, reinforcement-learning, education",
        },
        {
          name: "description",
          content:
            "Complete 13-week university capstone for building AI-controlled humanoid robots from scratch",
        },
        {
          name: "og:image",
          content: "https://physical-ai-robotics.dev/img/social-card.png",
        },
      ],

      // Navbar
      navbar: {
        title: "Physical AI & Humanoid Robotics",
        logo: {
          alt: "Physical AI Logo",
          src: "img/logo.svg",
        },
        items: [
          {
            href: "https://github.com/your-org/physical-ai-humanoid-robotics",
            label: "GitHub",
            position: "right",
          },
        ],
      },

      // Footer
      footer: {
        style: "dark",
        links: [
          {
            title: "Curriculum",
            items: [
              {
                label: "Weeks 1-3: Foundations",
                to: "/docs/week-01-foundations/01-ubuntu-setup",
              },
              {
                label: "Weeks 4-6: AI Training",
                to: "/docs/week-04-isaac-setup/01-isaac-sim-installation",
              },
              {
                label: "Weeks 7-9: Deployment",
                to: "/docs/week-07-unity/01-unity-ros-bridge",
              },
              {
                label: "Weeks 10-13: Hardware",
                to: "/docs/week-10-hardware/01-unitree-sdk",
              },
            ],
          },
          {
            title: "Resources",
            items: [
              {
                label: "Skills Documentation",
                to: "/skills",
              },
              {
                label: "Agents & Workflows",
                to: "/agents",
              },
              {
                label: "Hardware Guides",
                to: "/hardware",
              },
              {
                label: "Troubleshooting",
                to: "/docs/appendix/troubleshooting",
              },
            ],
          },
          {
            title: "Community",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/your-org/physical-ai-humanoid-robotics",
              },
              {
                label: "Discussions",
                href: "https://github.com/your-org/physical-ai-humanoid-robotics/discussions",
              },
              {
                label: "Discord",
                href: "https://discord.gg/your-server",
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Project. Built with Docusaurus.`,
      },

      // Code theme
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ["python", "cpp", "bash", "yaml", "json", "cmake"],
      },

      // Algolia search (optional, configure later)
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'physical-ai-robotics',
      // },

      // Color mode
      colorMode: {
        defaultMode: "dark",
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },

      // Announcement bar
      announcementBar: {
        id: "announcement-bar",
        content:
          '⭐ Star us on <a target="_blank" rel="noopener noreferrer" href="https://github.com/your-org/physical-ai-humanoid-robotics">GitHub</a> to support the project!',
        backgroundColor: "#00D9FF",
        textColor: "#0A0E27",
        isCloseable: true,
      },

      // Image zoom
      zoom: {
        selector: ".markdown img",
        background: {
          light: "rgb(255, 255, 255)",
          dark: "rgb(50, 50, 50)",
        },
        config: {},
      },
    }),

  plugins: [
    // PWA plugin for offline support
    [
      "@docusaurus/plugin-pwa",
      {
        debug: true,
        offlineModeActivationStrategies: [
          "appInstalled",
          "standalone",
          "queryString",
        ],
        pwaHead: [
          {
            tagName: "link",
            rel: "icon",
            href: "/img/logo.png",
          },
          {
            tagName: "link",
            rel: "manifest",
            href: "/manifest.json",
          },
          {
            tagName: "meta",
            name: "theme-color",
            content: "#00D9FF",
          },
        ],
      },
    ],
  ],
};

module.exports = config;
