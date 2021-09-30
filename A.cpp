#include<Siv3D.hpp> // OpenSiv3D v0.6.1
#include<Eigen/Core>
using namespace Eigen;
using namespace std;

class PPMInfo {
public:
	PPMInfo(StringView filepath) :
		m_reader(filepath) {
		if (not m_reader)return;
		m_reader.readLine();
		auto t = m_reader.readLine().value().split(U' ');
		divit.x = ParseInt<int>(t[1]);
		divit.y = ParseInt<int>(t[2]);
		t = m_reader.readLine().value().split(U' ');
		select_num = ParseInt<int>(t[1]);
		t = m_reader.readLine().value().split(U' ');
		cost.select = ParseInt<int>(t[1]);
		cost.swap = ParseInt<int>(t[2]);
	}
private:
	TextReader m_reader;
public:
	Size divit;
	int select_num;
	struct Cost {
		int select;
		int swap;
	}cost;
};

void Main()
{
	Window::SetStyle(WindowStyle::Sizable);

	constexpr StringView path = U"C:\\Users\\root\\source\\repos\\PLAN_A\\PLAN_A\\testcase\\problem2.ppm";

	Image image = PPMDecoder().decode(path);
	Texture texture(image);
	PPMInfo info(path);
	const Size tile_size = image.size() / info.divit;
	Grid<Image> tiles(info.divit, Image(tile_size));
	for (auto i : step(tiles.size())) {
		for (auto j : step(tile_size)) {
			tiles[i][j] = image[i * tile_size + j];
		}
	}

	Grid<s3d::Array<MatrixXd>> edge(tiles.size(), s3d::Array<MatrixXd>(4, MatrixXd(tile_size.x, 3)));
	for (auto i : step(edge.size())) {
		for (int j : step(tile_size.x)) {
			edge[i][0](j, 0) = tiles[i][0][j].r;
			edge[i][0](j, 1) = tiles[i][0][j].g;
			edge[i][0](j, 2) = tiles[i][0][j].b;
			edge[i][1](j, 0) = tiles[i][j][tile_size.x - 1].r;
			edge[i][1](j, 1) = tiles[i][j][tile_size.x - 1].g;
			edge[i][1](j, 2) = tiles[i][j][tile_size.x - 1].b;
			edge[i][2](j, 0) = tiles[i][tile_size.y - 1][tile_size.x - 1 - j].r;
			edge[i][2](j, 1) = tiles[i][tile_size.y - 1][tile_size.x - 1 - j].g;
			edge[i][2](j, 2) = tiles[i][tile_size.y - 1][tile_size.x - 1 - j].b;
			edge[i][3](j, 0) = tiles[i][tile_size.y - 1 - j][0].r;
			edge[i][3](j, 1) = tiles[i][tile_size.y - 1 - j][0].g;
			edge[i][3](j, 2) = tiles[i][tile_size.y - 1 - j][0].b;
		}
	}

	struct P {
		Point p1, p2;
		int r1, r2;
		double score;
	};
	s3d::Array<P> sl;
	for (auto i : step(tiles.size())) {
		for (auto j : step(tiles.size())) {
			if (i.x < j.x or i.x == j.x and i.y < j.y) {
				for(auto k : step(4)) {
					for (int l : step(4)) {
						double s = 0;
						for (int m : step(tile_size.x)) {
							s += (edge[i][k].row(m) - edge[j][l].row(m).reverse()).squaredNorm();
						}
						sl << P{ i,j,k,l,s };
					}
				}
			}
		}
	}
	sl.sort_by([](const auto& a, const auto& b) {return a.score < b.score; });
	{
		TextWriter writer(U"Banana.txt");
		for (const auto& i : sl)
			writer.writeln(U"{},{},{},{},{}"_fmt(i.p1, i.p2, i.r1, i.r2, i.score));
	}
	Image i(image.size());
	//tiles[sl[0].p1].overwrite(i, { 0,0 });
	//tiles[sl[1].p2].rotate270().overwrite(i, { tile_size.x,0 });
	DynamicTexture t(i);
	while (System::Update())
	{
		tiles[sl[0].p1].overwrite(i, { 0,0 });
		tiles[sl[0].p2].rotated(Math::HalfPi*(int)Scene::Time()).overwrite(i, { tile_size.x,0 });
		t.fill(i);
		t.draw();
	}
}
