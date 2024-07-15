#include <SFML/Graphics.hpp>
#include<iostream>
#include<queue>
#include<stack>
#include<chrono>

#include"imgui.h"
#include"imgui-SFML.h"

const int iScreenWidth = 720;
const int iMapSize = 20;
const float fVertSize = iScreenWidth / iMapSize;

bool AutoStep = false;
bool IsEditMode = true;

int iStartX = -1;
int iStartY = -1;
int iEndX = -1;
int iEndY = -1;
int iStep = 0;
int CurrentState = 0;
float FPS = 0;
float MouseClickTime = 10.0f;
float MouseClickTimer = 0.0f;
float StepInterval = 50.0f;
float StepCounter = 0.0f;


sf::Font TextFont;

class Vertex
{
public:
    sf::Vector2f vPosition;
    bool bVisited = false;
    bool bIsStarting = false;
    bool bIsEnding = false;
    bool bIsObstacle = false;
    float fCost = FLT_MAX;
    Vertex* parent = nullptr;
    std::vector<Vertex*> neighbours;

    Vertex()
    {
    }
    Vertex(sf::Vector2f vpos) : vPosition(vpos)
    {}
};

Vertex* Vertices = nullptr;
std::vector<Vertex*> Answer;

void Reset()
{
    delete[] Vertices;
    Answer.clear();
    CurrentState = 0;
    Vertices = new Vertex[iMapSize * iMapSize];
    iStartX = iStartY = -1;
    iEndX = iEndY = -1;
    for (int y = 0; y < iMapSize; y++)
        for (int x = 0; x < iMapSize; x++)
        {
            Vertices[y * iMapSize + x] = Vertex({ (float)x * fVertSize , (float)y * fVertSize });
            if (y > 0)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[(y - 1) * iMapSize + x]);
            if (y < iMapSize - 1)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[(y + 1) * iMapSize + x]);

            if (x > 0)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[y * iMapSize + (x - 1)]);
            if (x < iMapSize - 1)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[y * iMapSize + (x + 1)]);
        }
    iStep = 0;
}

void BreadthFirstSearch()
{
    CurrentState = 1;
    iStep = 0;
    if (iStartX == -1)
    {
        CurrentState = -1;
        std::cout << "[BFS] No Starting node\n";
        return;
    }
    if (iEndX == -1)
    {
        CurrentState = -2;
        std::cout << "[BFS] No Ending node\n";
        return;
    }
    for (int y = 0; y < iMapSize; y++)
        for (int x = 0; x < iMapSize; x++)
            Vertices[y * iMapSize + x].bVisited = false;
    Answer.clear();
    std::cout << "starting BFS \n";
    std::queue<Vertex*> Queue;
    std::stack<Vertex*> PathStack;
    Vertex* u = nullptr;

    Answer.push_back(&Vertices[iStartY * iMapSize + iStartX]);
    Vertices[iStartY * iMapSize + iStartX].bVisited = true;
    Queue.push(&Vertices[iStartY * iMapSize + iStartX]);
    PathStack.push(&Vertices[iStartY * iMapSize + iStartX]);
    while (!Queue.empty())
    {
        u = Queue.front();
        Queue.pop();
        if (u == &Vertices[iEndY * iMapSize + iEndX])
            break;
        for (int i = 0; i < u->neighbours.size(); i++)
        {   
            if (u->neighbours[i]->bVisited == false && u->neighbours[i]->bIsObstacle == false)
            {
                Answer.push_back(u->neighbours[i]);
                u->neighbours[i]->bVisited = true;
                Queue.push(u->neighbours[i]);
            }
        }
    }
    std::cout << "BFS completed " << Answer.size() << "\n";
}

void DepthFirstSearch()
{
    CurrentState = 2;
    iStep = 0;
    if (iStartX == -1)
    {
        CurrentState = -1;
        std::cout << "[DFS] No Starting node\n";
        return;
    }
    if (iEndX == -1)
    {
        CurrentState = -2;
        std::cout << "[DFS] No Ending node\n";
        return;
    }
    for (int y = 0; y < iMapSize; y++)
        for (int x = 0; x < iMapSize; x++)
            Vertices[y * iMapSize + x].bVisited = false;
    Answer.clear();
    std::cout << "starting DFS \n";

    Vertex* u = nullptr;
    std::stack<Vertex*> Stack;
    Answer.push_back(&Vertices[iStartY * iMapSize + iStartX]);
    Vertices[iStartY * iMapSize + iStartX].bVisited = true;
    for (int i = 0; i < Vertices[iStartY * iMapSize + iStartX].neighbours.size(); i++)
    {
        if (!Vertices[iStartY * iMapSize + iStartX].neighbours[i]->bIsObstacle)
            Stack.push(Vertices[iStartY * iMapSize + iStartX].neighbours[i]);
    }

    while (!Stack.empty())
    {
        u = Stack.top();
        Stack.pop();
        if (u->bVisited == false)
        {
            u->bVisited = true;
            Answer.push_back(u);
            if (u == &Vertices[iEndY * iMapSize + iEndX])
                break;
            for (int i = 0; i < u->neighbours.size(); i++)
            {
                if (u->neighbours[i]->bIsObstacle == false)
                {
                    Stack.push(u->neighbours[i]);
                }
            }
        }
    }
    std::cout << "DFS completed " << Answer.size() << "\n";
}

void DijkstraAlgo()
{
    CurrentState = 3;
    iStep = 0;
    if (iEndX == -1)
    {
        CurrentState = -2;
        std::cout << "[DIJKSTRA] No Ending node\n";
        return;
    }
    if (iStartX == -1)
    {
        CurrentState = -1;
        std::cout << "[DIJKSTRA] No Starting node\n";
        return;
    }
    for (int y = 0; y < iMapSize; y++)
        for (int x = 0; x < iMapSize; x++)
            Vertices[y * iMapSize + x].bVisited = false;
    Answer.clear();
    std::cout << "starting Dijkstra Algorithm \n";

    auto Distance = [](Vertex& v1, Vertex& v2) {
        return std::sqrt((v2.vPosition.x - v1.vPosition.x) * (v2.vPosition.x - v1.vPosition.x) + (v2.vPosition.y - v1.vPosition.y) * (v2.vPosition.y - v1.vPosition.y));
    };

    Vertex* CurrentVertex = &Vertices[iStartY * iMapSize + iStartX];
    Vertex* EndVertex = &Vertices[iEndY * iMapSize + iEndX];
    CurrentVertex->fCost = FLT_MIN;

    while(CurrentVertex != EndVertex)
    {
        for (auto vert : CurrentVertex->neighbours)
        {
            if (!vert->bVisited && !vert->bIsObstacle)
            {
                float EndDist = Distance(*EndVertex, *vert);
                vert->fCost = CurrentVertex->fCost + Distance(*vert, *CurrentVertex) + EndDist;
            }
        }
        std::sort(CurrentVertex->neighbours.begin(), CurrentVertex->neighbours.end(), [](Vertex* v1, Vertex* v2) { return v1->fCost < v2->fCost; });
        CurrentVertex->neighbours[0]->fCost += CurrentVertex->fCost;
        CurrentVertex->bVisited = true;
        
        for (int i = 0; i < CurrentVertex->neighbours.size(); i++)
            if (!CurrentVertex->neighbours[i]->bVisited && !CurrentVertex->neighbours[i]->bIsObstacle)
            {
                CurrentVertex = CurrentVertex->neighbours[i];
                break;
            }
        

        Answer.push_back(CurrentVertex);
        if (CurrentVertex == EndVertex)
            break;
    }

    
    std::cout << "Dijkstra algorithm completed " << Answer.size() << "\n";
}

void DrawText(sf::RenderWindow& win, sf::Font font, sf::Vector2f pos, std::string num, int nsize, sf::Color ccolor = sf::Color::White)
{
    sf::Text tex;

    tex.setFont(font);
    tex.setCharacterSize(nsize);
    tex.setString(num);
    tex.setFillColor(ccolor);
    tex.setPosition(pos);

    win.draw(tex);
}

void DrawLine(sf::Vector2f pos1, sf::Vector2f pos2, sf::RenderWindow& window)
{
    sf::Vertex line[2];
    line[0].position = pos1;
    line[0].color = sf::Color::White;
    line[1].position = pos2;
    line[1].color = sf::Color::White;

    window.draw(line, 2, sf::Lines);
}


int main()
{
    if (!TextFont.loadFromFile("Assets/Arialn.ttf"))
        std::cout << "[MAIN] Cannot load font\n";
    Vertices = new Vertex[iMapSize * iMapSize];
    for (int y = 0; y < iMapSize; y++)
        for (int x = 0; x < iMapSize; x++)
        {
            Vertices[y * iMapSize + x] = Vertex({ (float)x * fVertSize , (float)y * fVertSize });
            if (y > 0)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[(y - 1) * iMapSize + x]);  // UP 
            if (y < iMapSize - 1)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[(y + 1) * iMapSize + x]);  // DOWN

            if (x > 0)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[y * iMapSize + (x - 1)]);  // LEFT
            if (x < iMapSize - 1)
                Vertices[y * iMapSize + x].neighbours.push_back(&Vertices[y * iMapSize + (x + 1)]);  // RIGHT
        }

    sf::RenderWindow window(sf::VideoMode(iScreenWidth * 2, iScreenWidth), "Visualization");
    //window.setFramerateLimit(60);
    ImGui::SFML::Init(window);

    sf::Clock DeltaClock;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    while (window.isOpen())
    {
        start = std::chrono::high_resolution_clock::now();
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);
            if (event.type == sf::Event::Closed)
                window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
                window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Tab) && MouseClickTimer >= MouseClickTime)
            {
                MouseClickTimer = 0.0f;
                IsEditMode = !IsEditMode;
            }
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && MouseClickTimer >= MouseClickTime && IsEditMode)
            {
                MouseClickTimer = 0.0f;
                sf::Vector2i MousePos = sf::Mouse::getPosition(window);
                if (MousePos.x > 0 && MousePos.x < iScreenWidth && MousePos.y > 0 && MousePos.y < iScreenWidth)
                {
                    if (iStartX == -1 && sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) // selecting starting vertex
                    {
                        int x = MousePos.x / fVertSize;
                        int y = MousePos.y / fVertSize;
                        Vertices[y * iMapSize + x].bIsStarting = !Vertices[y * iMapSize + x].bIsStarting;
                        if (Vertices[y * iMapSize + x].bIsStarting) { iStartX = x; iStartY = y; }
                        else { iStartX = -1; iStartY = -1; }

                    }
                    else if (iEndX == -1 && sf::Keyboard::isKeyPressed(sf::Keyboard::LControl))  // selecting ending vertex
                    {
                        int x = MousePos.x / fVertSize;
                        int y = MousePos.y / fVertSize;
                        Vertices[y * iMapSize + x].bIsEnding = !Vertices[y * iMapSize + x].bIsEnding;
                        if (Vertices[y * iMapSize + x].bIsEnding) { iEndX = x; iEndY = y; }
                        else { iEndX = -1; iEndY = -1; }

                    }
                    else
                    {
                        int x = MousePos.x / fVertSize;
                        int y = MousePos.y / fVertSize;
                        Vertices[y * iMapSize + x].bIsObstacle = !Vertices[y * iMapSize + x].bIsObstacle;
                    }
                }
            }
        }
        ImGui::SFML::Update(window, DeltaClock.restart());
        window.clear(sf::Color(100, 100, 100));

        ImGui::Begin("Settings");
        ImGui::Text((IsEditMode) ? "Edit Mode" : "Sim Mode");
        ImGui::Checkbox("Toggle Edit mode", &IsEditMode);
        ImGui::Checkbox("Auto step", &AutoStep);
        ImGui::DragFloat("Auto step interval", &StepInterval, 1.0f, 0.1f, 1000.0f);
        if (ImGui::Button("Restart")) { iStep = 0; CurrentState = 0; }
        if (ImGui::Button("Reset"))
            Reset();

        ImGui::End();

        ImGui::Begin("Algorithm Menu");

        if (ImGui::Button("Step forward") && !IsEditMode)
        {
            if (iStep < Answer.size()) iStep++;
            else
                CurrentState = 4;
        }
        if (ImGui::Button("Breadth first search"))
            BreadthFirstSearch();
        if (ImGui::Button("Depth first search"))
            DepthFirstSearch();
        if (ImGui::Button("Djkstra's algorithm"))
            DijkstraAlgo();

        ImGui::End();

        sf::RectangleShape ConsoleRect;
        ConsoleRect.setPosition((float)iScreenWidth, 600.0f);
        ConsoleRect.setSize({ (float)iScreenWidth , iScreenWidth - 600.0f });
        ConsoleRect.setFillColor(sf::Color::Black);
        ConsoleRect.setOutlineThickness(3.0f);
        ConsoleRect.setOutlineColor(sf::Color(255, 0, 0));
        window.draw(ConsoleRect);

        for (int y = 0; y < iMapSize; y++)
            for (int x = 0; x < iMapSize; x++)
            {
                sf::RectangleShape square;
                square.setPosition(Vertices[y * iMapSize + x].vPosition);
                square.setSize({ fVertSize , fVertSize });
                square.setOutlineThickness(3.0f);
                square.setOutlineColor(sf::Color::Black);

                if (Vertices[y * iMapSize + x].bIsStarting)
                    square.setFillColor(sf::Color::Green);
                else if (Vertices[y * iMapSize + x].bIsEnding)
                    square.setFillColor(sf::Color::Red);
                else if (Vertices[y * iMapSize + x].bIsObstacle)
                    square.setFillColor(sf::Color(100, 100, 100));
                else
                    square.setFillColor(sf::Color(0, 0, 200));

                window.draw(square);
            }

        if (AutoStep && StepCounter >= 100.0f && !IsEditMode)
        {
            StepCounter = 0.0f;
            if (iStep < Answer.size()) iStep++;
            else
                CurrentState = 4;
        }

        if (Answer.size() > 0 && !IsEditMode)
            for (int i = 0; i < iStep; i++)
            {
                sf::RectangleShape square;
                square.setPosition({ (Answer[i]->vPosition.x + 5.0f) , (Answer[i]->vPosition.y + 5.0f) });
                square.setSize({ fVertSize - 12 , fVertSize - 12 });
                square.setOutlineThickness(2.0f);
                square.setOutlineColor(sf::Color::Black);
                square.setFillColor(sf::Color(200, 200, 200, 200));
                window.draw(square);
            }



        //DrawText(window, TextFont, { iScreenWidth + 50.0f , 30.0f }, (IsEditMode) ? "Edit Mode" : "Sim Mode", 16);
        switch (CurrentState)
        {
        case -1:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "ERROR : No starting node assigned", 32, sf::Color::Red);
            break;

        case -2:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "ERROR : No ending node assigned", 32, sf::Color::Red);
            break;

        case 1:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "BFS calculations done : " + std::to_string(Answer.size()), 32);
            break;

        case 2:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "DFS calculations done : " + std::to_string(Answer.size()), 32);
            break;

        case 3:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "Dijstra calculations done : " + std::to_string(Answer.size()), 32);
            break;

        case 4:
            DrawText(window, TextFont, { iScreenWidth + 50.0f , 620.0f }, "COMPLETE!!!", 32);
            break;

        default:
            break;
        }

        window.setTitle("Visualization FPS : " + std::to_string((int)FPS));

        ImGui::SFML::Render(window);
        window.display();

        MouseClickTimer++;
        StepCounter += StepInterval;

        // Calculating fps
        end = std::chrono::high_resolution_clock::now();
        FPS = (float)1e9 / (float)std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    }

    ImGui::SFML::Shutdown();
    delete[] Vertices;
    return 0;
}