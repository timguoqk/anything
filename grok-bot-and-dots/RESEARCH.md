# Grok Bot and OpenAI Dots: research notes

These notes back the interactive rebuild in [`index.html`](index.html). They cover what each product is, how its main UI is laid out, and where the screenshots came from. Research was done on Oct 8, 2026. Screenshots were reviewed locally and are not checked into this repo; every entry below links to its original source.

The rebuild only uses sample content: names, messages and numbers are made up and modeled on the screenshots, with a few lines quoting each company's own demo copy.

---

## 1. Grok Bot (SpaceXAI, formerly xAI)

### What it is

- **Launch:** Aug 11, 2026, in beta. The pitch: "Grok Bot is your team of always-on agents. They have their own computer, work inside tools and apps like you do, and keep working 24/7."
- **Bots:** Named, persistent agents with a job, memory, and routines. You message a Bot like a colleague. It works end to end and only comes back when something needs your approval.
- **Computer:** One persistent cloud computer per account, with a browser, filesystem and terminal. All your Bots share it, and each Bot gets its own screen on it. One Bot runs one computer-use task at a time. The FAQ warns not to treat separate Bots as a security boundary. The launch post says Bots "share a computer of their own in the cloud", while some press coverage describes one computer per Bot.
- **Teamwork:** Bots run in parallel, message each other, share channels (group chats), and hand work off. You can also tag @bot on X.
- **Teaching:** In the full-screen computer view, "Teach a task" lets a Bot watch you do a job once and then repeat it.
- **Plans:**
  - Included with every paid individual Cursor plan and Cursor Teams.
  - You can also link SuperGrok, SuperGrok Plus, SuperGrok Heavy, X Premium+, or a Grok Business seat.
  - Usage is separate from your Grok and Cursor allowances.
  - At launch it was SuperGrok, SuperGrok Plus and SuperGrok Heavy, plus Cursor Pro, Pro+, Ultra and Teams.
- **Apps:** Desktop on macOS, Windows and Linux, plus iOS and Android. Launch was macOS and iOS.
- **Around it:**
  - A Marketplace of shared Bots, organized by category, each with an Add button.
  - "Share as template" from a Bot's settings.
  - A plugins marketplace of MCP connectors.

### Main UI (desktop app, light theme)

| Region | What's there | Evidence |
|---|---|---|
| Sidebar (~300px, `#F4F4F4`) | Traffic lights and a "+" menu (New Bot / New Channel). Search field (`#E6E6E6`). Pinned Bots as large avatars with a gray label pill. Sections (e.g. Work, Ops, Unassigned) of rows: avatar, bold name, time, one-line preview. "Hidden Bots" with a count. Floating blue "More unreads" pill. Plugins. Account row with a menu (Get Grok Bot for iOS, Settings, About, Help Center, Send Feedback, Log out). | Grok Bot 101, Designing Grok Bot, multi-team guide |
| Row context menu | Pin; Move to (sections, New section); Mark as Unread; Edit Profile; Duplicate; Copy conversation ID | multi-team guide |
| Thread (`#FCFCFC`) | Header with the avatar and name. Bot bubbles are gray (`#E9E9E9`); your bubbles are black with white text. Also in the thread: centered timestamps, inline screenshots of the Bot's browser, A/B/C choice prompts that collapse to the picked answer with a check, task cards ("Running", "Open in Cursor"), draft approvals ("Send email" / "Discard"), plugin cards ("Authorize", "Added"), and system lines ("Renamed to…", "Created routine"). | 101 guide, support guide, App Store |
| Composer | Rounded field with a ⊕ button, placeholder "Message {Bot}" ("Ask {Bot}" on iPhone), and a black round mic button | 101 guide, App Store |
| Right panel (~316px, `#F9F9F9`) | Gear and ">>". "{Bot}'s screen" as a live thumbnail, or a monitor placeholder when idle. "Routines" with + (green clock icon, name, schedule). | 101 guide |
| Routine editor | Active toggle, Delete, Test run. Name, Instruction, "When to run" (+ Add another), and Run history with checks. | 101 guide |
| Settings | Avatar; Name; Label (optional); Description; Notifications ("Get notified when this Bot finishes / needs input"); Share as template | 101 guide, Designing guide |
| Profile popover | Large avatar on a gray card, description, Context (Instructions and memories), Integrations, "Team only" sharing, Copy link / Publish | Designing guide, templates guide |
| Cloud computer | Full-window dark overlay showing the Bot's Linux desktop (Chrome, dock with browser, tools and terminal). "Teach a task" and an exit button. | 101 guide |
| Plugins | Marketplace / Yours tabs, search, Featured (Gmail, Google Calendar, Google Drive, Granola…), categories, Added or Add | support guide |
| Channels | "# name" threads with several Bots, sender names over bubbles, and a Members panel with Add Member | multi-team guide |
| iPhone | Home: your avatar, search, + ; pinned Bots; list. Chat header: back, avatar-and-name pill, computer button. Approval cards. Computer view on black. | App Store |

**Avatars.** Flat solid shapes (circle, drop, cloud, squircle, rounded triangle, pill, hexagon, arch) with two white rounded "eyes", sometimes tilted or flat dashes. The palette is bright: orange `#FF9500`, pink `#FF3399`, blue `#1A82FF`, red `#FF2A3D`, gray `#B8B8B8`, green `#00C96B`, purple, teal and brown. Marketing shots put the Mac window on a coral gradient (`#FF8F92` → `#F4606C`).

### Screenshot sources

| Source | What it shows |
|---|---|
| [x.ai/bot/guides/grok-bot-101](https://x.ai/bot/guides/grok-bot-101) | The best set: the anatomy of the app (Grocery Bot, Routines, screen), the cloud computer with Teach a task, Arnold and pinned Bots, the routine editor, settings, search, the CRM and "outer loop" setups. Example image: [anatomy](https://media.x.ai/cdn-cgi/image/fit=scale-down,onerror=redirect,width=1280,height=800,f=auto/v1/website/grok-bot-guides-grok-bot-101-anatomy-2135357a.jpg) |
| [x.ai/bot/guides/designing-grok-bot-with-grok-bot](https://x.ai/bot/guides/designing-grok-bot-with-grok-bot) | Starter-bot cards (Chief of Staff, Inbox Triage, Commitment Tracker…), the tools picker, motion frames, the "meet the team" animation |
| [x.ai/bot/guides/how-i-run-multiple-teams-of-grok-bots](https://x.ai/bot/guides/how-i-run-multiple-teams-of-grok-bots) | New Bot / New Channel menu, channel Members panel, row context menu, iPhone group chats |
| [x.ai/bot/guides/templates-for-grok-bot](https://x.ai/bot/guides/templates-for-grok-bot) | Profile card, Share as template, Publish, the "View shared Bot" page |
| [x.ai/bot/guides/grok-bot-for-support](https://x.ai/bot/guides/grok-bot-for-support) | Plugins marketplace, Authorize cards, "Approve these write actions?", routines created from chat |
| [x.ai/bot/guides/grok-bot-for-engineering](https://x.ai/bot/guides/grok-bot-for-engineering) and [mobile app development](https://x.ai/bot/guides/grok-bot-for-mobile-app-development) | iPhone Bot list with labels, PR cards (View PR / Open in Cursor), Notion boards run by Bots |
| [Other role guides](https://x.ai/bot/guides) | GTM, PMs, recruiting, SDRs, legal ops, marketing, post-sales, founder-led sales |
| [App Store listing](https://apps.apple.com/us/app/grok-bot/id6794501026) | Five iPhone and four iPad screenshots: sign-in, "Work with many at once", "Signs in to your tools", "Works everywhere you do", "Comes back finished" |
| [Google Play listing](https://play.google.com/store/apps/details?id=ai.x.grok.bot) | Android screenshots |
| [Introducing Grok Bot](https://x.ai/news/introducing-grok-bot) | Launch post |
| [Docs: overview](https://docs.x.ai/grok-bot/overview), [get started](https://docs.x.ai/grok-bot/get-started), [FAQ](https://docs.x.ai/grok-bot/faq) | Product facts: shared computer, memory, platforms |
| [Cursor Help: plans](https://cursor.com/help/grok-bot/plans) | Plan matrix |
| [Marketplace](https://x.ai/bot/marketplace) | Shared Bots by category |
| [Christopher Alarcon: How to use Grok](https://christopheralarcon.com/blog/how-to-use-grok) | A real account walkthrough |
| [Unite.AI](https://www.unite.ai/xai-launches-grok-bot-always-on-ai-teammates-with-their-own-cloud-computers/), [Reworked](https://www.reworked.co/collaboration-productivity/xai-launches-grok-bot-ai-agents-in-beta/), [Digital Applied](https://www.digitalapplied.com/blog/grok-bot-ai-teammates-launch-cloud-computer-2026) | Launch coverage |

---

## 2. OpenAI Dots

### What it is

- **Launch:** Sep 29, 2026, announced at OpenAI DevDay. Pitched as "remarkably capable, always-on agents built to handle everything."
- **Model:** GPT-6 Astra.
- **Your dot:** Lives inside ChatGPT. You get one continuous thread with it and a plush 3D mascot you name and customize (shape, color, eyes, glasses, accessories, or a pixel "pet").
- **Computer and apps:** The dot has its own cloud computer (Linux, Chrome, files) and its own browser, and reaches 4,000+ apps through plugins. It works in the background and reports back.
- **Where you reach it:**
  - You create the dot at chatgpt.com/dots or in the ChatGPT desktop app (Mac or Windows).
  - After that, you can message it from the iOS and Android apps, Slack or Teams.
  - You can voice-call it.
  - Texting is a limited US beta.
- **Plans:**
  - Pro (outside the EEA, Switzerland and the UK) and Business Premium.
  - Enterprise, Edu and Healthcare when an admin enables it.
  - Talking to the dot doesn't count against ChatGPT limits; Codex or Work tasks it starts do.
- **Your Mac:** Optional. You "Allow" it, one Mac at a time, and only while the ChatGPT app is open.
- **Controls:**
  - Custom rules with four behaviors: take action without asking, take action when you say so, ask before taking action, hand off to you.
  - Plugin permissions: always ask, allow read-only actions, always allow low-risk actions.
  - Auto review of actions that touch your accounts.
  - Pause, Reboot and Delete from the dot's ••• menu.

### Main UI (ChatGPT desktop app, light theme)

| Region | What's there | Evidence |
|---|---|---|
| Window | Gray frame `#E5E6E7` with a 43px title bar: traffic lights, back/forward, sidebar toggle (with an unread dot), new chat; a bug icon and ••• at the right of the chat column | official composites |
| Rail (52px) | Home (filled, on a `#DBDCDD` tile), Library, Scheduled, Apps (@ with a plug), ••• , divider, Explore, account | official composites; labels partly inferred |
| Thread | The mascot (~80px) floats over a white name pill holding the name and a live status ("Thinking…", "Clarifying the experiment goal"), with a 💡 while it thinks. Dot bubbles are `#F3F3F3`. Your bubbles take the dot's color (Felipe `#4165E8`, Alfred `#F5CF69`, Todd `#B8D648`, Iggy `#E978A9`, Jojo `#DD6ADC`). Also: emoji reaction badges on bubble corners, "Read 4:35 PM", "↳" quote lines, thread chips with counts, underlined links joined with " · " | composites, MacStories, Anima |
| Header buttons | Three round 36px white buttons: open computer, call, toggle panel | composites |
| Composer | 54px pill: +, "Send a message", mic, and a send button tinted with the dot's color | composites |
| Side card / profile | Avatar with a pencil badge, name, status; Call and Slack; Computers ("{name}'s computer — Connected" with a live thumbnail, "Your computer — Allow"); Recent activity (threads with unread dots, scheduled items with cadence); Outputs; ••• menu (Rename, Pause, Reboot, Delete) | MacStories, PromptsLove |
| Computer tab | "{name}'s computer" tab. Remote Chrome inside a frame filled with a gradient of the dot's color. A new-tab page with the mascot ("This is my computer. Watch me work, or take control when you need to."), a clock, "Welcome back, {name}", and an app grid. Dock. Pill: "{name} has control · Take over" / "You have control · Return control". | MacStories, Anima, official |
| Customize your dot | Colors (ring swatches), Characters, Pets (pixel art), each paged with ‹ ›; a large name field, a 3D preview, Save. A build-your-own flow lets you pick shape, eyes, glasses and accessories (crown, bow tie, beret, bucket hat…). | MacStories, PromptsLove, AI Agents Library, launch film |
| Onboarding | "Create your dot" with three benefits, then Continue. Then "Choose Google plugins to install" (toggles, Continue to Google / Continue without…). Then connect your computer (desktop only). Then the dot introduces itself: "Hey! I'm your dot." … "Want to give me a name?" and a "Customize your dot ›" chip. | AI Agents Library, Anima, help center |
| Settings › Personalization › Permissions › Custom rules | "When {name} wants to…" / "{name} should…", with four behaviors | OpenAI safety video, help center |
| Calls | Full-screen in the dot's color: timer, Speaker / End / Mute | launch film |
| Phone | Round ≡ and phone buttons, the mascot with a name pill, a sidebar with "Your dot / Scheduled / Library / Customize / Explore", and a yellow "Chat" button | MacStories, launch film |

**Characters.** The mascots have a fuzzy plush finish:

| Character | Look |
|---|---|
| Felipe | Blue cloud with a black beret |
| Alfred | Yellow rounded triangle with round glasses, closed "‿" eyes and a bow tie |
| Todd | Lime frog with googly eyes and a bow tie |
| Jojo | Magenta heart with sunglasses |
| Iggy | Pink ball with headphones and a smile |
| Puck | Teal bunny with headphones and sunglasses (from a MacStories account) |
| Default | Gray "O" ring named "dot" |

### Screenshot sources

| Source | What it shows |
|---|---|
| [OpenAI: Introducing dots](https://openai.com/index/introducing-dots/) | Five 4K composites (proposal/Alfred, analysis/Todd, feedback/Iggy, launch/Felipe, content/Jojo on iPhone) and the cloud computer view. The images are on `images.ctfassets.net`. |
| [MacStories: a first look](https://www.macstories.net/stories/openais-dots-a-first-look/) | Real desktop screenshots (side card, Customize modal, Linux VM) and the iPhone (dark chat, ••• menu, sidebar) |
| [Anima: OpenAI Dots in ChatGPT](https://animaapp.com/blog/agentic/openai-dots/) | chatgpt.com/dots first run in light mode; the computer pane with Return control |
| [PromptsLove: OpenAI Dots guide](https://promptslove.com/blog/openai-dots-guide/) | Dark mode: Customize modal, profile popover, ••• menu, help-center captures |
| [AI Agents Library: OpenAI Dots review](https://www.aiagentslibrary.com/blog/openai-dots-review/) | Create your dot, Google plugin picker, naming, the accessory grid and palette |
| [Product Hunt: Dots by OpenAI](https://www.producthunt.com/products/dots-by-openai) | Gallery: plugins, cloud computer, Activity view, Slack thread |
| [VKTR / Reworked: DevDay announcements](https://www.vktr.com/ai-news/the-biggest-announcements-from-openai-devday-dots-agents-new-500-plan/) | Activity panel ("In progress" / "Past activity") and the mobile profile from the keynote |
| [OpenAI Help: Getting started with your dot](https://help.openai.com/en/articles/20001530-getting-started-with-your-dot) and [privacy, security and safety FAQ](https://help.openai.com/en/articles/20001529-dots-privacy-security-and-safety-faqs) | Setup, controls and rules wording |
| [9to5Google](https://9to5google.com/2026/09/29/openai-dots-agent/), [The New Stack](https://thenewstack.io/openai-dots-gpt6-agents/), [TechCrunch](https://techcrunch.com/2026/09/29/openai-launches-dots-its-bubbly-agentic-avatar/), [Tom's Guide](https://www.tomsguide.com/ai/openai-just-launched-dots-heres-why-most-chatgpt-users-cant-try-them-yet) | Re-hosted official images, keynote slides (specialist dots), and a computer-view frame |
| OpenAI videos (Vimeo 1231177365, 1231093676, 1230606080) | Launch film (customization, calls, computer), safety video (mobile Custom rules and plugin permissions), blog demo (mobile chat, lock-screen notification) |
| [TestingCatalog: pre-launch leak](https://www.testingcatalog.com/openai-develops-platform-for-always-on-agents-on-chatgpt/) | The April "Agents" precursor UI. This is not the shipped Dots UI and was not used for the rebuild. |

**Avoid:** an AI-generated "OpenAI dot" laptop image circulating on how-to sites is fake UI.

---

## 3. Confirmed vs inferred in the rebuild

- **Confirmed by screenshots:**
  - Grok Bot: every surface in its table above, except the recorded-steps list and the "learned" dialog after Teach a task.
  - Dots: the thread, side card, computer tab, Customize modal, onboarding screens, calls and phone layout.
- **Inferred:**
  - Grok Bot: the recorded-steps list and "learned" dialog after Teach a task.
  - Dots:
    - The desktop layout of the Custom rules settings. The wording is OpenAI's; the screens shown were mobile.
    - Approval cards in the thread. The docs describe approvals but no screenshot shows the card.
    - The Scheduled, Library and Apps views.
    - The "Connect this Mac?" onboarding screen.
    - Exact rail labels.
- **Simulated:** All agent behavior (replies, tasks, routines, approvals) runs locally from canned rules. Nothing is sent anywhere.
