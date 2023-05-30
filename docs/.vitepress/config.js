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
          { text: "Descrição do robô, sua finalidade e aplicações", link: "/pages/introduction/description.md" },
          { text: "Visão geral dos componentes eletrônicos, mecânicos e de software", link: "/pages/introduction/vision.md" },
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
          { text: "Instruções de montagem e conexão dos componentes", link: "/pages/electronics/instructions.md" },
        ],
      },
      {
        text: "Estrutura Mecânica",
        collapsible: false,
        items: [
          { text: "Introduction", link: "/pages/mechanics/introduction.md" },
          { text: "Materiais utilizados na construção", link: "/pages/mechanics/materials.md" },
          { text: "Projetos CAD e desenhos técnicos", link: "/pages/mechanics/designs.md" },
          { text: "Instruções de montagem e configuração mecânica", link: "/pages/mechanics/assembly.md" },
        ],
      },
      {
        text: "Software e Ambiente ROS",
        collapsible: false,
        items: [
          { text: "Introdução ao ROS (Robot Operating System) e sua importância", link: "/pages/software/introduction.md" },
          { text: "Configuração do ambiente de desenvolvimento ROS", link: "/pages/software/settings.md" },
          { text: "Integração com o pacote Nav2 para navegação autônoma", link: "/pages/software/integration.md" },
          { text: "Descrição dos pacotes ROS utilizados no robô", link: "/pages/software/description.md" },
        ],
      },
      {
        text: "Configuração e Parametrização",
        collapsible: false,
        items: [
          { text: "Configuração do ambiente ROS e pacotes relacionados", link: "/pages/parameterization/environment.md" },
          { text: "Configuração de sensores e atuadores", link: "/pages/parameterization/sensors.md" },
          { text: "Parametrização dos algoritmos de navegação e planejamento de trajetória", link: "/pages/parameterization/algorithms.md" },
        ],
      },
      {
        text: "Operação e Manutenção",
        collapsible: false,
        items: [
          { text: "Instruções de inicialização e desligamento do robô", link: "/pages/maintenance/instructions.md" },
          { text: "Guia de operação para usuários finais", link: "/pages/maintenance/guide.md" },
          { text: "Procedimentos de manutenção preventiva e corretiva", link: "/pages/maintenance/procedures.md" },
          { text: "Solução de problemas e resolução de problemas comuns", link: "/pages/maintenance/solution.md" },
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