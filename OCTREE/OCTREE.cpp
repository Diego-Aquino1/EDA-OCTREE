#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPointData.h>
#include <vtkGlyph3D.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

struct Point {
    float x, y, z;
    Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};

struct OctreeNode {
    Point* point;
    OctreeNode* children[8]; // 8 children nodes

    OctreeNode() {
        point = nullptr;
        for (int i = 0; i < 8; ++i) {
            children[i] = nullptr;
        }
    }
};

class Octree {
private:
    OctreeNode* root;

    // Función auxiliar para insertar un punto en un nodo del Octree
    void insertHelper(OctreeNode* node, Point* p, float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        // Verificar si el nodo actual es nulo
        if (node == nullptr) {
            node = new OctreeNode();
        }

        // Si el nodo actual está en una hoja (sin hijos), insertar el punto aquí
        if (node->point == nullptr) {
            node->point = p;
            return;
        }

        // Calcular el centro del nodo actual
        float centerX = (minX + maxX) / 2.0f;
        float centerY = (minY + maxY) / 2.0f;
        float centerZ = (minZ + maxZ) / 2.0f;

        // Determinar en qué octante cae el punto
        int octant = 0;
        if (p->x >= centerX) octant |= 1;
        if (p->y >= centerY) octant |= 2;
        if (p->z >= centerZ) octant |= 4;

        // Calcular los límites del octante
        float newMinX = (octant & 1) ? centerX : minX;
        float newMinY = (octant & 2) ? centerY : minY;
        float newMinZ = (octant & 4) ? centerZ : minZ;
        float newMaxX = (octant & 1) ? maxX : centerX;
        float newMaxY = (octant & 2) ? maxY : centerY;
        float newMaxZ = (octant & 4) ? maxZ : centerZ;

        // Insertar el punto en el hijo correspondiente
        insertHelper(node->children[octant], p, newMinX, newMinY, newMinZ, newMaxX, newMaxY, newMaxZ);
    }

    // Función auxiliar para verificar si un punto existe en un nodo del Octree
    bool existHelper(OctreeNode* node, Point* p, float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
        if (node == nullptr) {
            return false;
        }

        // Si el nodo actual tiene un punto y coincide con el punto que buscamos, retornar verdadero
        if (node->point != nullptr && node->point->x == p->x && node->point->y == p->y && node->point->z == p->z) {
            return true;
        }

        // Calcular el centro del nodo actual
        float centerX = (minX + maxX) / 2.0f;
        float centerY = (minY + maxY) / 2.0f;
        float centerZ = (minZ + maxZ) / 2.0f;

        // Determinar en qué octante podría estar el punto
        int octant = 0;
        if (p->x >= centerX) octant |= 1;
        if (p->y >= centerY) octant |= 2;
        if (p->z >= centerZ) octant |= 4;

        // Calcular los límites del octante
        float newMinX = (octant & 1) ? centerX : minX;
        float newMinY = (octant & 2) ? centerY : minY;
        float newMinZ = (octant & 4) ? centerZ : minZ;
        float newMaxX = (octant & 1) ? maxX : centerX;
        float newMaxY = (octant & 2) ? maxY : centerY;
        float newMaxZ = (octant & 4) ? maxZ : centerZ;

        // Realizar búsqueda recursiva en el hijo correspondiente
        return existHelper(node->children[octant], p, newMinX, newMinY, newMinZ, newMaxX, newMaxY, newMaxZ);
    }
public:
    Octree() {
        root = new OctreeNode();
    }

    void insert(Point* p) {
        insertHelper(root, p, -FLT_MAX, -FLT_MAX, -FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
    }

    bool exist(Point* p) {
        return existHelper(root, p, -FLT_MAX, -FLT_MAX, -FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX);
    }
};

void readPointsFromFile(const std::string& filename, std::vector<Point>& points) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        float x, y, z;
        if (std::getline(ss, token, ',') &&
            std::getline(ss, token, ',') && (std::istringstream(token) >> x) &&
            std::getline(ss, token, ',') && (std::istringstream(token) >> y) &&
            std::getline(ss, token) && (std::istringstream(token) >> z)) {
            points.emplace_back(x, y, z);
        }
    }
    file.close();
}

int main() {
    std::vector<Point> points;
    readPointsFromFile("points1.csv", points); 
    Octree octree;

    // Insertar los puntos en el Octree
    for (const auto& point : points) {
        Point* p = new Point(point.x, point.y, point.z);
        octree.insert(p);
    }

    // Visualización con VTK
    vtkSmartPointer<vtkPoints> vtkPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (const auto& point : points) {
        vtkPoints->InsertNextPoint(point.x, point.y, point.z);
        vtkIdType id[1];
        id[0] = vtkPoints->GetNumberOfPoints() - 1;
        vertices->InsertNextCell(1, id);
    }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(vtkPoints);
    polydata->SetVerts(vertices);

    vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName("points.vtp"); // Nombre del archivo de salida
    writer->SetInputData(polydata);
    writer->Write();

    // Código para visualizar los puntos utilizando VTK
    vtkSmartPointer<vtkPoints> vtkPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

    for (const auto& point : points) {
        vtkPoints->InsertNextPoint(point.x, point.y, point.z);
        vtkIdType id[1];
        id[0] = vtkPoints->GetNumberOfPoints() - 1;
        vertices->InsertNextCell(1, id);
    }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(vtkPoints);
    polydata->SetVerts(vertices);

    vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    writer->SetFileName("points.vtp"); // Nombre del archivo de salida
    writer->SetInputData(polydata);
    writer->Write();

    // Visualización de los puntos con VTK
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polydata);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);

    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(800, 600);

    vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    renderWindow->AddRenderer(renderer);

    interactor->Initialize();
    interactor->Start();

    return 0;
}