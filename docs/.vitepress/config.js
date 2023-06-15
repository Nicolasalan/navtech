export default {
  title: "Adocs",
  description: "An awesome docs template built by me",

  themeConfig: {
    logo: "/logo.svg",
    siteTitle: "Docs",
    // Navbar Link
    nav: [
      { text: "About", link: "/about" },
      { text: "Contact", link: "/contact" },
      { text: "Guide", link: "/guide" },
    ],
    // Social Icons
    socialLinks: [
      { icon: "github", link: "https://github.com/Nicolasalan/Navtech" },
    ],
    // Sidebar
    sidebar: [
      {
        text: "Introdução e Visão Geral do Robô",
        collapsible: true,
        items: [
          { text: "Descrição do robô, sua finalidade e aplicações", link: "/pages/introduction/instructions.md" },
          { text: "Visão geral dos componentes eletrônicos, mecânicos e de software", link: "/pages/introduction/description.md" },
          { text: "Requisitos e especificações do sistema", link: "/pages/introduction/requirements.md" },
        ],
      },
      {
        text: "Componentes Eletrônicos",
        collapsible: true,
        items: [
          { text: "Lista de componentes eletrônicos utilizados no robô", link: "/pages/electronics/list.md" },
          { text: "Diagramas de circuito", link: "/pages/electronics/diagrams.md" },
          { text: "Especificações dos componentes e funções", link: "/pages/electronics/specifications.md" },
        ],
      },
      {
        text: "Estrutura Mecânica",
        collapsible: false,
        items: [
          { text: "Projetos CAD e desenhos técnicos", link: "/pages/mechanics/designs.md" },
        ],
      },
      {
        text: "Software e Ambiente ROS",
        collapsible: false,
        items: [
          { text: "Introdução ao ROS (Robot Operating System) e sua importância", link: "/pages/software/introduction.md" },
          { text: "Configuração do ambiente de desenvolvimento ROS", link: "/pages/software/settings.md" },
          { text: "Integração com o pacote Nav2 para navegação autônoma", link: "/pages/software/guide.md" },
          { text: "Descrição dos pacotes ROS utilizados no robô", link: "/pages/software/description.md" },
        ],
      },
    ],
    footer: {
      message: "Released under the MIT License.",
      copyright: "Copyright © 2022-present Adocs",
    },
    markdown: {
      theme: "material-palenight",
      lineNumbers: true,
    },
  },
};