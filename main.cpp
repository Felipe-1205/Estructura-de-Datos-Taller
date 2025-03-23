#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>
#include <fstream>
using namespace std;

template <class T, class U>
class Grafo
{
private:
    vector<T> vertices; // lista de vértices que proporciona el índice en la matriz
    U **aristas;             // matriz de adyacencia con doble puntero (memoria dinámica)
public:
    Grafo()
    {
        aristas = nullptr;
    }
    void setVertices(vector<T> v)
    {
        vertices = v;
    }
    void setAristas(U **a)
    {
        aristas = a;
    }
    vector<T> getVertices()
    {
        return vertices;
    }
    U **getAristas()
    {
        return aristas;
    }
    int cantVertices()
    {
        return vertices.size();
    }
    int cantAristas()
    {
        int suma = 0;
        for (int i = 0; i < cantVertices(); i++)
        {
            for (int j = 0; j < cantVertices(); j++)
            {
                if (*(*(aristas + i) + j) != 0)
                    suma++;
            }
        }
        return suma;
    }
    int buscarVertice(T vert)
    {
        int ind = -1;
        for (int i = 0; i < cantVertices(); i++)
        {
            if (vertices[i] == vert)
                ind = i;
        }
        return ind;
    }
    bool insertarVertice(T vert)
    {
        bool res = false;
        if (buscarVertice(vert) == -1)
        {
            vertices.push_back(vert);
            U **nmatriz = new U *[cantVertices()];
            for (int i = 0; i < cantVertices(); i++)
            {
                nmatriz[i] = new U[cantVertices()];
            }
            for (int i = 0; i < cantVertices() - 1; i++)
            {
                for (int j = 0; j < cantVertices() - 1; j++)
                {
                    nmatriz[i][j] = aristas[i][j];
                }
            }
            for (int i = 0; i < cantVertices(); i++)
            {
                nmatriz[i][cantVertices() - 1] = 0;
                nmatriz[cantVertices() - 1][i] = 0;
            }
            for (int i = 0; i < cantVertices() - 1; i++)
            {
                delete[] aristas[i];
            }
            delete[] aristas;
            aristas = nmatriz;
            res = true;
        }
        return res;
    }   
    bool insertarArista(T ori, T des, U cos)
    {
        bool res = false;
        int i_ori = buscarVertice(ori);
        int i_des = buscarVertice(des);
        if (i_ori != -1 && i_des != -1)
        {
            if (aristas[i_ori][i_des] == 0)
            {
                aristas[i_ori][i_des] = cos;
                res = true;
            }
        }
        return res;
    }
    U buscarArista(T ori, T des)
    {
        U res = -1;
        int i_ori = buscarVertice(ori);
        int i_des = buscarVertice(des);
        if (i_ori != -1 && i_des != -1)
        {
            res = aristas[i_ori][i_des];
        }
        return res;
    }
    bool eliminarArista(T ori, T des)
    {
        bool res = false;
        int i_ori = buscarVertice(ori);
        int i_des = buscarVertice(des);
        if (i_ori != -1 && i_des != -1)
        {
            aristas[i_ori][i_des] = 0;
            res = true;
        }
        return res;
    }
    vector<T> recorrerGrafo()
    {
        vector<T> camino; // Vector que almacenará el camino recorrido
        vector<bool> visitado(cantVertices(), false); // Vector para marcar los vértices visitados
        int indiceActual = 0; // Índice del vértice actual

        visitado[indiceActual] = true; // Marcar el primer vértice como visitado
        camino.push_back(vertices[indiceActual]); // Agregar el primer vértice al camino

        // Realizar el recorrido hasta visitar todos los vértices
        while (camino.size() < cantVertices())
        {
            U distanciaMinima = numeric_limits<U>::max(); // Inicializar la distancia mínima con un valor máximo
            int siguienteIndice = -1; // Índice del siguiente vértice a visitar

            // Buscar la arista de menor distancia desde el vértice actual
            for (int i = 0; i < cantVertices(); i++)
            {
                if (!visitado[i] && aristas[indiceActual][i] != 0 && aristas[indiceActual][i] < distanciaMinima)
                {
                    distanciaMinima = aristas[indiceActual][i];
                    siguienteIndice = i;
                }
            }
            // Si se encontró una arista válida, avanzar al siguiente vértice
            if (siguienteIndice != -1)
            {
                indiceActual = siguienteIndice;
                visitado[indiceActual] = true;
                camino.push_back(vertices[indiceActual]);
            }
            else
            {
                // Si no se encontró una arista válida, significa que no hay más caminos posibles
                break;
            }
        }

        return camino;
    }
};

class Point {
public:
    double x;
    double y;

    bool operator==(const Point& other) const {
        return (x == other.x) && (y == other.y);
    }
};

double calculateDistance(const Point& p1, const Point& p2)
{
    double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
    return distance;
}
int main()
{
    ifstream inputFile("in_0.txt");
    ofstream outputFile("output.txt");

    int n;
    inputFile >> n;
    outputFile << n << endl;

    for (int i = 0; i < n; i++)
    {
        Grafo<Point, double> miGrafo;
        int m;
        inputFile >> m;
        outputFile << m << endl;

        Point points;
        points.x=0;
        points.y=0;
        miGrafo.insertarVertice(points);
        for (int j = 0; j < m; j++)
        {
            inputFile >> points.x >> points.y;
            miGrafo.insertarVertice(points);
        }
        vector<Point> vertices=miGrafo.getVertices();
        for (int i = 0; i < m+1; i++)
        {
            for (int j = 0; j < m+1; j++)
            {
                if(i!=j){
                    miGrafo.insertarArista(vertices[i],vertices[j],calculateDistance(vertices[i], vertices[j]));
                }
            }
        }
        vector<Point> ordenado=miGrafo.recorrerGrafo();
        for (int j = 1; j < m+1; j++)
        {
            outputFile << ordenado[j].x << " " << ordenado[j].y << endl;
        }
    }

    inputFile.close();
    outputFile.close();

    return 0;
}
