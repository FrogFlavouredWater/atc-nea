# Contents {#contents .TOC-Heading}

[1. Analysis [2](#analysis)](#analysis)

[2. Outline of Project [2](#outline-of-project)](#outline-of-project)

[3. Stakeholders [3](#stakeholders)](#stakeholders)

[4. How can the problem be solved by computational methods? [3](#how-can-the-problem-be-solved-by-computational-methods)](#how-can-the-problem-be-solved-by-computational-methods)

[5. Thinking Abstractly [4](#thinking-abstractly)](#thinking-abstractly)

[6. Thinking Ahead [4](#thinking-ahead)](#thinking-ahead)

[7. Thinking Procedurally [5](#thinking-procedurally)](#thinking-procedurally)

[8. Thinking Concurrently [6](#thinking-concurrently)](#thinking-concurrently)

[Conclusion [7](#conclusion)](#conclusion)

[Research: [7](#research)](#research)

[Interview With Harry Bremner: [7](#interview-with-harry-bremner)](#interview-with-harry-bremner)

[Interview Plan: [7](#interview-plan)](#interview-plan)

[Questions: [8](#questions)](#questions)

[Existing Product Research: [13](#existing-product-research)](#existing-product-research)

[EUROCONTROL's ESCAPE [13](#eurocontrols-escape)](#eurocontrols-escape)

[User Interface: [14](#user-interface)](#user-interface)

[NASA Ames FACET (research/systems-analysis tool) [16](#nasa-ames-facet-researchsystems-analysis-tool)](#nasa-ames-facet-researchsystems-analysis-tool)

[User Interface: [17](#user-interface-1)](#user-interface-1)

[Specific Details: [21](#specific-details)](#specific-details)

[Design: [25](#design)](#design)

[Systems Diagram: [25](#systems-diagram)](#systems-diagram)

[Justification of Each Module: [25](#justification-of-each-module)](#justification-of-each-module)

[Flowcharts: [28](#flowcharts)](#flowcharts)

[Class Diagram: [32](#class-diagrams)](#class-diagrams)

# 1. Analysis {#analysis}

Air traffic control (ATC) is a very safety critical field, which needs sequencing, scheduling, and separation of multiple aircraft. ATC uses procedural and radar based methods to keep aircraft apart and maintain separation. Due to the computerised nature of the field already, it makes sense for this project to be created for computers. In my project, I will be creating an automated air traffic control resolver, which will utilise these algorithms to control an airspace. Aircraft will be represented as nodes moving in 3D space, however will be rendered on a flat screen as a node in 2D, with an altitude value displayed indicating their height in the z axis, I will then use vectoring algorithms which determine efficient routes, and conflict resolution algorithms can enforce safe separation.

Existing simulators (such as Eurocontrol's ESCAPE Software) show that computational approaches are commonly used for this sort of project. However, most examples focus either on entertainment or on very technical industry training. This project will sit between either end, and be an educational tool that shows how algorithms (like vectoring, scheduling, optimisation, and conflict resolution) can be integrated into a simplified ATC scope.

## 2. Outline of Project {#outline-of-project}

The program will be written in C++ and will simulate multiple aircraft approaching a single runway. There may later be the option of allowing . Each aircraft will determine the shortest path, dynamically making adjustments to prevent conflicts. A scheduling layer will reduce holding times and balance the arrivals. The system will include a simple graphical interface displaying aircraft tracks, alerts, runways and other important information

## 3. Stakeholders {#stakeholders}

The main stakeholders for this project are A Level Computer Science students aged 16--18. This group represents the target audience, as the simulator is designed to show how algorithms like vectoring algorithms, scheduling, and optimisation can be used in real world safety critical problems. By using the radar style UI and seeing how aircraft adjust their paths dynamically, the students understand both the concepts and the practical application in aviation or computer science. This age group is also relevant because they are developing both their programming skills and their ability to think computationally, while also being the next demographic to enter the workplace.

Teachers could also use the program as a demonstration tool during lessons. The simulator can visually show scheduling algorithms, and abstraction, which makes it easier to visualise.

Overall, the project is aimed at stakeholders who need an educational and accessible model of air traffic management. For my project, the stakeholder who I will be interviewing, is Harry Bremner, an 18 year old computer science student with an interest in aviation.

## 4. How can the problem be solved by computational methods? {#how-can-the-problem-be-solved-by-computational-methods}

Air traffic management can be shown in terms of data structures and algorithms, which makes it very well suited to computational methods. Each aircraft will be rendered as an object (using concepts from object oriented programming) with attributes like position, speed, heading, and altitude. The airspace itself can be modelled as a graph of waypoints and edges, allowing shortest-path algorithms to calculate efficient routes to the runway. The program will detect potential conflicts by comparing the distance and altitude between pairs of aircraft, and will then implement the most appropriate resolution strategy, for instance: adjusting speed, heading, or altitude. Scheduling aircraft to land close to their planned arrival times can be modelled as an optimisation problem, and the system will attempt to minimise delays and excessive manoeuvring.

The radar visualisation is also very computationally suitable. By updating aircraft positions frame by frame, the program can use simple a simple graphics library (In this project it will be C++ RayLib) to draw range rings, headings, and identifiers on screen, which will give real time feedback on how the algorithms work. This will make a constant cycle of

"Input -\> Algorithm -\> Output"

Which is the same way that real world ATC systems operate. Because the whole program can be broken down into smaller computational tasks, such as vectoring algorithms, separation checks, scheduling, and visualisation it is very well suited to a structured programming language like C++.

### 5. Thinking Abstractly {#thinking-abstractly}

Abstraction is important, because it lets programmers show complex systems in a way that simplifies both the development process, and the user's experience. Each aircraft is abstracted into an object with attributes like position, heading, altitude, and speed and they are encapsulated within a class to represent its current state and behaviour. The airspace will be abstracted into a matrix of nodes acting as waypoints and possible routes, forming a graph structure that algorithms can traverse efficiently. Real world complex and difficult tasks like weather effects and delays in communication will be excluded to make sure it is computationally viable, and simple to allow for ease of use. Instead, it will focus on parts like spatial movement, conflict detection, and scheduling. The abstraction makes sure that the system is understandable, and efficient.

### 6. Thinking Ahead {#thinking-ahead}

In order to make the project feasible, and easily buildable, I will take a modular approach, and separate the structure into classes and files to containerise the program. The simulator will be structured so that new features, like multiple runways, different aircraft types, or more advanced algorithms (like predictive conflict management) can be added without having to restructure the whole program. By keeping the computational complexity in the vectoring algorithm and conflict detection low, the program can handle more aircraft without losing a significant amount of performance.

This is also one reason why C++ has been chosen, due to its speed, and efficiency in memory usage, which makes it good for real time calculations and rendering. I will also anticipate any future additions to the project, with separate modules for data handling, vectoring, scheduling, and rendering, which will let me reuse code and make debugging easier.

### 7. Thinking Procedurally {#thinking-procedurally}

**Input**

The simulation runs as a loop that takes aircraft updates and new arrivals. Each aircraft is instantiated as an object of a class, and states such as position, heading, speed, and altitude are all stored.

**Vectoring**

Once aircraft states are handled, the program will compute the route to the runway using a vectoring stage. The vectors must handle the aircraft speed and turn limits, while avoiding drawing a vector over the actual runway itself in order to respect restricted boundaries.

**Visualisation**

The updated state gets rendered to a radar style interface, which shows range rings, labels, vectors and alerts so the users can see the consequences of the choices that the algorithm has made in real time.

**Conflict detection**

After the vectors are set, the system will check for potential 'loss of separation' (LoS) or a conflict. The aircraft will be checked to see if their vectors cross within a certain distance, and if they do, it will issue a "traffic advisory" caution. This will log the incident, but not perform any kind of resolution. If this is not enough, for example the aircraft are already too close, the next step is a "resolution advisory" where the algorithm will provide a vector for one or both aircraft in order to resolve the conflict as soon as possible. If a conflict is predicted, the resolution will be applied in order of least disruption: speed adjustment, then heading change, then altitude change.

This makes sure that the aircraft won't collide, as the predictive step reduces the chance of a last second resolution and should flag and correct conflicts early.

**Scheduling & prioritisation**

The scheduling will decide the landing order which should be both fair and efficient. It should minimise any deviation from the target landing times and avoid excessive holding. The scheduler can insert short holding patterns, and resequence aircraft when needed.

### 8. Thinking Concurrently {#thinking-concurrently}

Concurrency is important for real world air traffic control solutions, however for the purpose of my program it will not be required. This is due to the fact that the language and libraries that I have selected are both very fast for logical calculations, meaning it will be able to be handled on one thread. If I were to add parallel processing, or concurrent operations I would then have to correct for desync errors.

## Conclusion

In conclusion, my project is well-suited for a computational approach, and it can use computational methods to resolve many of the complicated problems. If this program is successfully built using a computer program, it will result in a tool capable of resolving air traffic conflicts, which can respond dynamically, and in real time to aircraft's positions and projected paths.

# Research:  {#research}

## Interview With Harry Bremner:

### Interview Plan:

The interview will be used to attempt to find what requirements the users need, and what sort of ideas I will need to take onboard before creating the project. The aim is to find what features and design elements should be included in the simulator to make it fulfil its success criteria.

The main three areas of focus for this interview are:

- **What makes an effective simulator**

- **What features are most important**

- **What kind of visuals and interface are preferred.**

Some smaller questions were made under these headings to make the requirements as specific as possible and to guide development of the system.

### Questions:

**What makes an effective simulator?**

• What do you think makes a simulator educational and engaging?

It should teach something while still being interactive. I like when you can see what's happening in real time rather than just reading raw data.

• How realistic should the simulation be?

It doesn't need to be perfectly realistic, but it should behave in a way that makes sense. The program can be abstracted as much as necessary to make it understandable and performant.

• How important is accuracy compared to performance?

For me, accuracy in behaviour matters more than perfect physics. The algorithms need to feel believable, but the program should run smoothly without lag.

• Would you prefer the simulator to focus more on algorithms or on recreating the ATC environment?

Definitely more on the algorithms showing how pathfinding and scheduling work. The ATC theme just makes it more interesting.

• Should the simulator include explanations or tooltips that describe what the algorithms are doing?

Yes, that would help a lot. Something that shows what the algorithm is currently calculating or which aircraft it's adjusting would make it easier to follow and learn from.

**What features are most important?**

• Should the user be able to adjust simulation parameters (like number of aircraft or separation distance)?

Yes, that would be useful. It lets you test different conditions and see how the algorithms handle them.

• How much control should the user have over the aircraft?

I think it should be mostly automated, but maybe you could override decisions or click an aircraft to view its route and speed. That would make it interactive without being too complex.

• Would it be useful to include a pause, slow-down, or fast-forward function for testing?

Definitely. That's something most simulators need, especially if you're trying to analyse what's happening.

• Would you like the program to show performance metrics, such as delay time or number of conflicts avoided?

Yes, because it gives measurable results. You could then compare different algorithms or scenarios to see which works best.

• Should the program allow saving and replaying scenarios?

That would be really helpful for reviewing tests. It's not essential for the first version, but a replay feature would make it more professional.

**What kind of visuals and interface are preferred?**

• What type of interface do you think suits this project best?

A radar-style display with moving aircraft icons and labels. It should be clean and easy to read rather than overcomplicated.

• Should the aircraft icons display extra information like altitude or call sign?

Yes, altitude and heading would be useful, but other metrics should be hidden, or shown only when zoomed in or selected, so it doesn't get cluttered.

• Would you prefer a dark or light theme for the interface?

Dark theme, like a real radar screen. It's easier on the eyes and looks more professional.

• Should there be alerts or colour changes when a conflict is detected?

Definitely. Maybe aircraft turn red or flash when they're too close which would make conflicts obvious.

• Would sound effects (like warning tones or notifications) make it better or distracting?

A few subtle sounds would be good, like a ping when there's a conflict, but nothing constant or loud.

**Interview Script - 02/11/25**

**Stakeholder:** *Harry Bremner, 18-year-old Computer Science student with an interest in aviation*  
**Interviewer:** *Hugo Woolrich-Burt*

**Review of the Interview**

This interview showed what the main stakeholders expect from the simulator. The responses showed that the simulator should prioritise algorithms and interactivity over full realism. The focus should be on understanding pathfinding, conflict resolution, and scheduling rather than simulating every technical aspect of real ATC systems.

In terms of design, a radar-style dark interface with minimal graphics is preferred. Some key requirements are:

- real-time movement

- visible conflict alerts

- optional metric displays such as average delay or total conflicts avoided.

Features like simulation speed controls and adjustable parameters are seen as important for learning and testing.

The interview proved that the simulator should be educational, interactive, and efficient. And gave useful guidance for the development process.

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Key User Requirements</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><table>
<colgroup>
<col style="width: 14%" />
<col style="width: 43%" />
<col style="width: 42%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Requirement No.</strong></th>
<th><strong>Requirement Description</strong></th>
<th><strong>Justification / Source (from Interview)</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>1</strong></td>
<td>The simulator must show aircraft moving in real time on a radar-style display.</td>
<td>The stakeholder said that seeing aircraft move “in real time” makes it more engaging and helps visualise the algorithms at work.</td>
</tr>
<tr class="even">
<td><strong>2</strong></td>
<td>The interface should use a dark theme similar to real radar screens.</td>
<td>The user preferred a dark background because it looks professional and is easier on the eyes.</td>
</tr>
<tr class="odd">
<td><strong>3</strong></td>
<td>Each aircraft must be represented as an object with attributes such as position, speed, heading, and altitude.</td>
<td>Required to support accurate simulation and visualisation; user wants logical and believable aircraft movement.</td>
</tr>
<tr class="even">
<td><strong>4</strong></td>
<td>The program should include a pathfinding algorithm to route aircraft to the runway efficiently.</td>
<td>The stakeholder was mainly interested in learning how algorithms are applied to real-world problems.</td>
</tr>
<tr class="odd">
<td><strong>5</strong></td>
<td>The system must include a conflict detection routine that identifies aircraft breaching minimum separation.</td>
<td>The stakeholder said aircraft should behave “logically,” and conflict alerts would make the simulation realistic and useful.</td>
</tr>
<tr class="even">
<td><strong>6</strong></td>
<td>The simulator should automatically apply conflict resolution rules (speed, heading, or altitude changes).</td>
<td>Ensures aircraft adjust dynamically, as discussed when describing resolution stages.</td>
</tr>
<tr class="odd">
<td><strong>7</strong></td>
<td>The simulator should have a basic scheduling system that prioritises aircraft by planned arrival time.</td>
<td>The stakeholder wanted the system to show algorithmic decision-making and fairness.</td>
</tr>
<tr class="even">
<td><strong>8</strong></td>
<td>Users must be able to adjust simulation parameters such as number of aircraft or minimum separation distance.</td>
<td>The stakeholder said this would make it more interactive and allow testing of different conditions.</td>
</tr>
<tr class="odd">
<td><strong>9</strong></td>
<td>The simulator should allow pausing, slowing down, or speeding up the simulation.</td>
<td>The stakeholder wanted to be able to analyse events at different speeds for learning.</td>
</tr>
<tr class="even">
<td><strong>10</strong></td>
<td>The interface should display visual alerts (e.g., colour change or flash) when conflicts occur.</td>
<td>The stakeholder suggested aircraft “turn red or flash when they’re too close.”</td>
</tr>
<tr class="odd">
<td><strong>11</strong></td>
<td>Optional subtle sound alerts should play when conflicts are detected.</td>
<td>The stakeholder said “a few subtle sounds would be good,” improving immersion without distraction.</td>
</tr>
<tr class="even">
<td><strong>12</strong></td>
<td>Aircraft labels should show useful data (altitude, heading, ID) when selected or zoomed in.</td>
<td>Helps readability and prevents clutter — requested in the visuals section.</td>
</tr>
<tr class="odd">
<td><strong>13</strong></td>
<td>The program should display simple metrics such as total delay time or number of conflicts avoided.</td>
<td>The stakeholder said metrics help evaluate algorithm performance.</td>
</tr>
<tr class="even">
<td><strong>14</strong></td>
<td>The system should be performant enough to handle multiple aircraft smoothly.</td>
<td>The stakeholder prioritised smooth performance over perfect realism.</td>
</tr>
<tr class="odd">
<td><strong>15</strong></td>
<td>The codebase should be modular to allow later features like replays or multi-runway scenarios.</td>
<td>Suggested as an optional extension and to support scalability in future development.</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

# Existing Product Research:

To determine the position and features that I want in my simulator, I decided to review 3 very different air traffic control programs, which all fit different niches. The applications are:

- EUROCONTROL: ESCAPE

- FACET -- Future Air traffic management Concepts Evaluation Tool (NASA)

- Planes Control - (ATC)

These each perform a different function, spanning from real time interconnected flight controllers, to air traffic management style games for entertainment.

## [EUROCONTROL's ESCAPE]{.underline}  {#eurocontrols-escape}

ESCAPE is a real time air traffic management (ATM) simulation platform which is used by EUROCONTROL and most of Europe's air traffic controllers as a study, experiment and training program. The program supports complicated scenarios, along with real time weather, traffic, and unexpected events such as failures. It also has the ability to connect to flight simulators at the same time, allowing pilots to receive training simultaneously, while also being able to introduce some level of human error to the aircraft, making the training much more valuable to the controllers.

ESCAPE is built with a modular approach, with separate vectoring, conflict detection/resolution, scheduling and display subsystems, with a scenario based design, which makes its core structure similar to my project.

Due to the level of complexity of ESCAPE, it has very high system requirements, often requiring multiple servers to run each local zone, which makes it much less accessible to the individual as a training or learning aid, which is an area that my project will pick up on, and attempt to be as universally functional as possible.

### User Interface:

![](media/image1.png){width="6.5in" height="2.9521981627296587in"}

- The UI shows aircraft with colours to help identify aircraft, along with adding visual hierarchy which separates the elements.

- It uses simple graphics to ensure that the display isn't too cluttered, which keeps things simple

![A screenshot of a computer
AI-generated content may be incorrect.](media/image2.png){width="7.306737751531059in" height="3.3020833333333335in"}

- An aircraft's track can be seen highlighted in a contrasting colour when selected, which makes it

![](media/image3.png){width="7.257950568678915in" height="3.2916666666666665in"}

- The UI uses simple shapes and lines to abstract complicated features such as map details, and unnecessary information which would only serve to complicate the view

- I dislike how old and cluttered the UI looks, with poor contrast and visibility in some cases, however it is reasonable due to the lack of importance that a clean UI has when compared to the relative importance of designing a secure and fail proof safety system.

![A computer screen shot of a map
AI-generated content may be incorrect.](media/image4.png){width="6.4in" height="3.0407699037620297in"}

- ESCAPE also has a map-based view where more details can be shown, with a navigable viewport which shows a map of the region, with flight paths overlaid and waypoints visible. This is embedded in a table style workspace with raw data.

- This is good, as just raw data is very hard to analyse and work from, therefore allowing controllers to visualise the aircraft in space can improve their workflow.

- Again, the UI is not very intuitive and user friendly.

## NASA Ames FACET (research/systems-analysis tool)

NASA's Future ATM Concepts Evaluation Tool (FACET) is a professional air traffic management simulator used for large scale research and testing of future airspace concepts. It was developed by NASA Ames Research Center for modelling and analysing the movement of thousands of aircraft simultaneously within U.S. airspace.

NASA claims 'FACET can evaluate route optimisation, conflict detection, and flow management strategies, allowing researchers to test how new algorithms or procedures might improve safety and efficiency before they are implemented in real life'

FACET is mainly built for research rather than education, meaning it focuses on performance and accuracy rather than accessibility or user experience. It uses high level mathematical models and advanced optimisation techniques to predict and manage air traffic flow on a national scale.

FACET also is not good at simulating the small / local airspace clusters, and rather is used for simulating large scale air traffic, which means it is less suited for my task.

However, many of the ideas behind FACET are relevant to my project. Both programs use pathfinding, conflict detection, and optimisation algorithms to manage air traffic safely and efficiently. FACET also tracks metrics such as delay time, route deviation, and the number of conflicts resolved, which I will replicate on a smaller scale to give users meaningful data about how well the simulation performed.

### User Interface:

![A screen shot of a computer
AI-generated content may be incorrect.](media/image5.png){width="6.489583333333333in" height="5.3125in"}

- The old versions of the simulator were black and white with very minimal graphics, using very fast libraries, for maximum simulation performance on very old machines

- They abstracted a huge amount of detail, relying on text based buttons, and simple lines to show information

> ![A map of the united states
> AI-generated content may be incorrect.](media/image6.png){width="6.5in" height="4.46875in"}

- This becomes very complicated when viewing from a national scale, and can make it hard to understand.

- The software is therefore very specialised and specific to single tasks, and is not suited to many applications such as education.

![A screenshot of a computer
AI-generated content may be incorrect.](media/image7.png){width="6.597498906386702in" height="4.229166666666667in"}

- The later versions of FACET then added colour based segmentation to the viewport along with being able to simulate and render more complicated features such as weather, and flight plan routes

![](media/image8.png){width="6.5in" height="4.072916666666667in"}

- The software now has many different viewports which all show different data to the controllers, who can navigate its interface in a much more intuitive way.

- For example, the flight details sector reuses familiar buttons and UI element styling to make sense to the user, where as the flight list section uses a familiar list element

![A map of the united states
AI-generated content may be incorrect.](media/image9.png){width="6.656944444444444in" height="4.118055555555555in"}

- The software now shows many colours and is customizable to let those monitoring the simulation have greater control over how the program operates and displays data

# Specific Details:

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Hardware Requirements</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><table>
<colgroup>
<col style="width: 17%" />
<col style="width: 33%" />
<col style="width: 48%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Requirement No.</strong></th>
<th><strong>Hardware Specification</strong></th>
<th><strong>Justification / Purpose</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>1</strong></td>
<td><strong>Processor (Minimum):</strong> Intel i5 or AMD Ryzen 5 (quad-core)</td>
<td>Provides enough CPU performance to calculate multiple aircraft positions and run pathfinding algorithms smoothly.</td>
</tr>
<tr class="even">
<td><strong>2</strong></td>
<td><strong>Processor (Recommended):</strong> Intel i7 or AMD Ryzen 7</td>
<td>Ensures consistently high frame rates when simulating larger numbers of aircraft or higher-resolution radar visuals.</td>
</tr>
<tr class="odd">
<td><strong>3</strong></td>
<td><strong>Memory (RAM):</strong> 8 GB minimum, 16 GB recommended</td>
<td>Required for storing aircraft objects, pathfinding graphs, and visual buffers without stuttering or data loss.</td>
</tr>
<tr class="even">
<td><strong>4</strong></td>
<td><strong>Graphics:</strong> Integrated GPU (Intel UHD 620 / AMD Vega) minimum</td>
<td>The Raylib graphics library relies on basic GPU acceleration; integrated graphics are sufficient for 2D radar rendering.</td>
</tr>
<tr class="odd">
<td><strong>5</strong></td>
<td><strong>Storage:</strong> 500 MB free disk space</td>
<td>Stores the program executable, resource files, and any saved simulation data or configuration files.</td>
</tr>
<tr class="even">
<td><strong>6</strong></td>
<td><strong>Display:</strong> 1080p monitor (1920×1080)</td>
<td>Provides enough resolution to clearly display multiple aircraft, range rings, and text labels without overlapping.</td>
</tr>
<tr class="odd">
<td><strong>7</strong></td>
<td><strong>Input Devices:</strong> Keyboard and mouse</td>
<td>Needed for user control — adjusting parameters, pausing the simulation, and selecting aircraft.</td>
</tr>
<tr class="even">
<td><strong>8</strong></td>
<td><strong>Audio:</strong> Standard sound card / speakers</td>
<td>Required for subtle conflict-alert tones or notification sounds.</td>
</tr>
<tr class="odd">
<td><strong>9</strong></td>
<td><strong>Operating System:</strong> Windows 10 or later (64-bit)</td>
<td>Compatible with the C++ compiler, Raylib library, and CLion/Visual Studio development environments.</td>
</tr>
<tr class="even">
<td><strong>10</strong></td>
<td><strong>Optional Hardware:</strong> Dual monitors</td>
<td>Allows one screen to display the simulation and the other to show algorithm data or debug output during testing.</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Software Requirements</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><table>
<colgroup>
<col style="width: 17%" />
<col style="width: 32%" />
<col style="width: 50%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Requirement No.</strong></th>
<th><strong>Software / Hardware Requirement</strong></th>
<th><strong>Justification / Purpose</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>1</strong></td>
<td><strong>Language:</strong> C++</td>
<td>Chosen for its speed and efficiency, which are important for handling multiple aircraft updates in real time.</td>
</tr>
<tr class="even">
<td><strong>2</strong></td>
<td><strong>IDE:</strong> CLion or Visual Studio</td>
<td>Provides strong debugging tools, syntax highlighting, and CMake integration for C++ projects.</td>
</tr>
<tr class="odd">
<td><strong>3</strong></td>
<td><strong>Graphics Library:</strong> Raylib</td>
<td>Lightweight and easy-to-use C++ graphics library used to draw radar visuals, range rings, and aircraft icons.</td>
</tr>
<tr class="even">
<td><strong>4</strong></td>
<td><strong>Operating System:</strong> Windows 10 or later</td>
<td>Compatible with the chosen development tools and hardware; provides a stable runtime environment.</td>
</tr>
<tr class="odd">
<td><strong>5</strong></td>
<td><strong>Hardware:</strong> Intel i5 / Ryzen 5 or above, 8GB RAM minimum</td>
<td>Ensures smooth simulation performance and real-time rendering without frame drops.</td>
</tr>
<tr class="even">
<td><strong>6</strong></td>
<td><strong>Compiler:</strong> MinGW or MSVC</td>
<td>Required to compile and link C++ source code into an executable program.</td>
</tr>
<tr class="odd">
<td><strong>7</strong></td>
<td><strong>Version Control:</strong> Git / GitHub</td>
<td>Allows version tracking, backup, and collaborative changes during development.</td>
</tr>
<tr class="even">
<td><strong>8</strong></td>
<td><strong>File Format:</strong> JSON or CSV</td>
<td>Used for storing configuration data such as aircraft schedules and settings.</td>
</tr>
<tr class="odd">
<td><strong>9</strong></td>
<td><strong>Input Devices:</strong> Keyboard and Mouse</td>
<td>Required for user interaction, such as pausing, zooming, or selecting aircraft on the radar.</td>
</tr>
<tr class="even">
<td><strong>10</strong></td>
<td><strong>Output Device:</strong> Monitor (minimum 1080p resolution)</td>
<td>Displays the radar interface, aircraft icons, and visual alerts clearly.</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

<table>
<colgroup>
<col style="width: 100%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Success Criteria</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><table>
<colgroup>
<col style="width: 12%" />
<col style="width: 43%" />
<col style="width: 44%" />
</colgroup>
<thead>
<tr class="header">
<th><strong>Criterion No.</strong></th>
<th><strong>Success Criterion</strong></th>
<th><strong>How It Will Be Tested / Measured</strong></th>
</tr>
</thead>
<tbody>
<tr class="odd">
<td><strong>1</strong></td>
<td>Aircraft appear on the radar and move smoothly in real time.</td>
<td>Observe the simulation — there should be no visible lag or frame skipping.</td>
</tr>
<tr class="even">
<td><strong>2</strong></td>
<td>The program accurately detects conflicts when aircraft come within the set minimum separation distance.</td>
<td>Create test scenarios with known separation breaches and check that alerts trigger correctly.</td>
</tr>
<tr class="odd">
<td><strong>3</strong></td>
<td>The system automatically resolves conflicts by adjusting heading, altitude, or speed.</td>
<td>Run controlled conflict scenarios and verify aircraft adjust their paths to restore safe separation.</td>
</tr>
<tr class="even">
<td><strong>4</strong></td>
<td>Pathfinding algorithms (e.g., A* or Dijkstra) calculate efficient routes to the runway.</td>
<td>Compare generated routes against expected shortest paths using test datasets.</td>
</tr>
<tr class="odd">
<td><strong>5</strong></td>
<td>Scheduling system prioritises aircraft according to planned arrival times.</td>
<td>Create multiple arrival scenarios and confirm the order matches expected scheduling.</td>
</tr>
<tr class="even">
<td><strong>6</strong></td>
<td>The interface clearly displays aircraft position, labels, and range rings on a radar-style layout.</td>
<td>Visually inspect radar output and confirm all key data (heading, altitude, ID) is visible and correctly positioned.</td>
</tr>
<tr class="odd">
<td><strong>7</strong></td>
<td>Conflict alerts appear visually (aircraft turns red or flashes) and optionally play a sound.</td>
<td>Trigger controlled breaches and verify both the visual and audible alerts activate.</td>
</tr>
<tr class="even">
<td><strong>8</strong></td>
<td>User can adjust simulation parameters such as number of aircraft or separation distance.</td>
<td>Change settings and confirm the updated values affect the simulation in real time.</td>
</tr>
<tr class="odd">
<td><strong>9</strong></td>
<td>Simulation can be paused, slowed down, or sped up without errors.</td>
<td>Test time controls during runtime and confirm consistent program behaviour.</td>
</tr>
<tr class="even">
<td><strong>10</strong></td>
<td>Program maintains stable performance with multiple aircraft (20+ simultaneously).</td>
<td>Run a high-load scenario and confirm consistent frame rate and responsiveness.</td>
</tr>
<tr class="odd">
<td><strong>11</strong></td>
<td>Delay and efficiency metrics (e.g., average delay time) are displayed after each run.</td>
<td>Run multiple tests and check the metrics update and display correctly.</td>
</tr>
<tr class="even">
<td><strong>12</strong></td>
<td>The interface remains clear and uncluttered, even with many aircraft on screen.</td>
<td>Evaluate readability during dense traffic conditions and verify no overlap or distortion.</td>
</tr>
<tr class="odd">
<td><strong>13</strong></td>
<td>Users can observe algorithmic processes (e.g., current path, conflict resolution) during runtime.</td>
<td>Display debug overlays or algorithm status indicators and verify accuracy.</td>
</tr>
<tr class="even">
<td><strong>14</strong></td>
<td>No crashes or runtime errors occur during normal use.</td>
<td>Conduct extended runtime tests to ensure long-term stability.</td>
</tr>
<tr class="odd">
<td><strong>15</strong></td>
<td>The program runs on standard classroom PCs without installation issues.</td>
<td>Test deployment on multiple Windows systems to confirm compatibility and full functionality.</td>
</tr>
</tbody>
</table></td>
</tr>
</tbody>
</table>

# Design:  {#design}

## Systems Diagram:

![](media/image10.png){width="7.60536854768154in" height="3.5444444444444443in"}

Using a top down approach, and building the related subsystems for the project, means that I can break the program down into smaller components, which makes the design, development and testing all easier. This is because it will allow me to compartmentalize each individual section into a function or series of functions which can be debugged faster. I will be able to visualize the inputs and outputs, allowing for a much faster and smoother development process.

## Justification of Each Module:

**[Startup:]{.underline}**

[Initialise viewport:]{.underline}

- Run the constructor functions to build the window using the graphics library

- Begin drawing to the GPU buffer

- Start all critical functions related to window management and other prerequisites for the program

[Load options from file]{.underline}

- Pull the data from the options file, saved to project root, or %APPDATA% folder

- Load the options and run the sim based on the flags inputted

- Eg. Simulation speed, target framerate, startup configurations, etc

[Load settings from file]{.underline}

- Load keybinds from the file

- Start listener service for keypresses, and pass them off to the program

**[Menu:]{.underline}**

[Pause]{.underline}

- Render when the user presses \[esc\], show a GUI with options to control the program

- Halt all program operations, and freeze time, to allow the user to change settings.

- Use a performant library such as RayGUI to render the interface

- Save changes to file

[Settings options]{.underline}

- Save all keybind changes to the file

**[Simulating:]{.underline}**

[Rendering:]{.underline}

- Display the updated locations of the aircraft

- Render the viewport background and any elements on the screen such as waypoints and runways

- Render the current state of any aircraft (eg. red range rings around aircraft which are within minimum separation)

- Show the current aircraft data, such as the information about the altitude, heading etc.

[Logic:]{.underline}

[Conflict Detection:]{.underline}

- Draw a path along the aircraft's current vector, and detect if it collides with another vector.

- If aircraft experience a loss of separation, run a resolution algorithm to divert them, and send one into a holding pattern if required

[Spawn Aircraft:]{.underline}

- Allow the user to place aircraft, which will automatically be handled by the computer's algorithm

- Have a soft limit of aircraft so the user doesn't crash the program, or make the airspace too unusable, however allow the user to bypass this if they really want.

[Detect Aircraft on ILS]{.underline}

- Check to see if the aircraft is at a low altitude and aligned with the localiser. If it is, then allow it to be handed off to the ground controller, who will manage the landing (Automate the aircraft's landing from that point forward.

[Input:]{.underline}

- Run a listener service for user keypresses

- Hand off the data to the service that requires it

[Simulation End:]{.underline}

- Gather all required data and results from the simulation

- Present the data to the user in a clean format

- Show comparisons with other scenarios, to show what the computer is best at

## Flowcharts:

![](media/image11.png){width="5.140660542432196in" height="4.936111111111111in"}![](media/image12.png){width="1.707159886264217in" height="4.9363637357830275in"}**Conflict Detection: Conflict Resolution:**

**[Program Structure Flowcharts:]{.underline}**

![](media/image13.png){width="3.3164555993000877in" height="3.997509842519685in"}**Startup:**

**Menu:**

![](media/image14.png){width="3.841030183727034in" height="4.113923884514436in"}

**Rendering:**

![](media/image15.png){width="6.583951224846894in" height="8.291139545056868in"}

**Spawn Aircraft:**

![](media/image16.png){width="4.177215660542432in" height="4.177215660542432in"}

![](media/image17.png){width="3.7006846019247592in" height="4.565332458442695in"}**Detect aircraft on ILS approach:**

## Class Diagrams:

**Engine, UI, & Simulation:**

![](media/image18.png){width="3.2911329833770777in" height="4.208333333333333in"}

Engine is the top-level controller for the whole program, so its main job is to coordinate everything else rather than contain the detailed logic itself. This design choice is beneficial because it gives the project one clear entry point for the application lifecycle. In the code, the Engine class is responsible for constructing the window, applying settings, managing the main loop, handling high-level input, and switching between major program states such as the menu, the running simulation, and the pause screen. It also stores the currently selected aircraft, which allows the rest of the program to respond to user interaction without making the UI or simulation directly manage global state.

This is good object-oriented design because it reduces coupling. This is because the UI class does not need to know how the application starts or stops, and the Simulation class does not need to know about menus or window setup. The singleton pattern used here also reflects the fact that there should only ever be one main engine controlling the program. Engine makes the rest of the codebase easier to manage by delegating the specialist tasks to UI and Simulation.

Simulation is the core backend class which models all of the autonomous air traffic control logic itself. Its role is to store and update the aircraft and airports, apply autonomous behaviour, process commands, detect conflicts, manage spawning, and determine when aircraft have landed or left the simulation area. This allows it to simulate the actual airspace, as if it were real without any attachment to the rest of the simulation, which means it could run headlessly if required. Rather than scattering aircraft movement, spawning, and conflict checking across multiple unrelated files, the Simulation class contains all of the related methods and variables for this. To ensure it does not become a "god class" it does not try to do absolutely everything on its own, because it makes use of helper classes such as SpawnService, TrajectoryPredictor, and GuidancePreviewService. This is a strong design decision because it keeps Simulation as the main controller of the domain while still delegating specialist calculations to more focused classes.

UI is the presentation layer of the system, and its purpose is to display information to the user without owning the core simulation logic. This separation is important because it means the appearance of the program and the behaviour of the simulation are not tightly bound. In this project, the UI class is responsible for drawing the main menu, settings menu, pause screen, simulation HUD, airports, aircraft, trails, localizer visuals, and guidance previews. It also performs coordinate conversion between nautical miles and screen pixels, which is a logical responsibility for a user interface class since that conversion is directly related to rendering. The class is justified because it provides a clean boundary between backend state and frontend display. It reads information from the Simulation object and presents it visually, but it does not decide how aircraft move, when conflicts occur, or how autonomous landing logic works. That keeps the design modular and easier to maintain. The UI class exists to improve maintainability and clarity by isolating all display-related behaviour into one place. This follows good software engineering practice because changes to visuals, menus, or HUD layout can be made without rewriting the simulation logic, while changes to aircraft behaviour can be made without affecting how the interface is drawn.

**Aircraft & Airport**

![](media/image19.png){width="3.2588156167979in" height="4.221009405074366in"}

Aircraft represents an individual plane in the simulation and stores all of the data needed to control and update it. This includes its motion state, command targets, callsign, flight phase, control mode, and conflict status. The class is justified because each aircraft needs to behave as its own object with its own position, speed, altitude, and instructions. By keeping this data and behaviour together, the program can update each aircraft independently while still allowing the simulation to check separation, issue commands, and guide aircraft toward landing.

Airport represents a runway location and the approach information associated with it. It stores the airport name, position, runway heading, runway length, and localizer data used for ILS capture. This class is justified because the airport is a key object in the landing system: aircraft need a fixed point to navigate toward, and the simulation needs a structured way to test whether an aircraft is inside the localizer signal area. Keeping this data in its own class makes the landing logic clearer and avoids hardcoding runway information directly into the simulation.

**AircraftPerformance, MotionState & Command**

![](media/image20.png){width="3.9738057742782154in" height="4.64077646544182in"}

## Evaluation:
