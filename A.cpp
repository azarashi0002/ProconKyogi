#include<Siv3D.hpp> // OpenSiv3D v0.6.1
#include<Eigen/Core>
using namespace Eigen;
using namespace std;

Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ImageToMatrix(const Image& image) {
	Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> ret(image.width(), image.height());
	for (const auto [x, y] : step(image.size())) {
		const auto [r, g, b, a] = ColorF(image[y][x]);
		ret(y, x) << r, g, b;
	}
	return ret;
}
std::array<Eigen::MatrixX3d, 4> PickEdge(const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& panel) {
	std::array<Eigen::MatrixX3d, 4> ret;
	std::array<Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, 1>, 4> edge = {
		panel.topRows(1).row(0).transpose().eval(),
		panel.rightCols(1).col(0).eval(),
		panel.bottomRows(1).row(0).transpose().reverse().eval(),
		panel.leftCols(1).col(0).reverse().eval()
	};

	for (int i : step(4)) {
		Eigen::MatrixX3d mat(panel.rows(), 3);
		for (int j : step(panel.rows())) {
			mat.row(j) = edge[i][j];
		}
		ret[i] = mat;
	}
	return ret;
}
int GetP2Rotate(int p1r, int p1e, int p2e) {
	return (p1r + p1e + p2e + 2) % 4;
}

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

struct Graph {
	Point p;
	int r;
};

void Main()
{
	Window::SetStyle(WindowStyle::Sizable);

	constexpr StringView path = U"testcase/problem2.ppm";

	const Image image = PPMDecoder().decode(path);
	const Texture texture(image);
	const PPMInfo info(path);
	const Size tile_size = image.size() / info.divit;
	const Grid<Image> tiles = Grid<Image>::IndexedGenerate(info.divit, [&](const Point& p) {return image.clipped(Rect(p * tile_size, tile_size)); });
	const auto mat_tiles = (tiles >> [](const auto& image) {return ImageToMatrix(image); });
	const Grid<std::array<Eigen::MatrixX3d, 4>> edge = (mat_tiles >> [](const auto& mat) {return PickEdge(mat); });

	struct P {
		Point p1, p2;
		int r1, r2;
		double score;
	};
	const auto score_calc = [&](const Point& p1, int d1, const Point& p2, int d2) {
		return (edge[p1][d1] - edge[p2][d2].colwise().reverse()).rowwise().squaredNorm().sum();
	};
	s3d::Array<P> scorearray;
	for (auto i : step(tiles.size())) {
		for (auto j : step(tiles.size())) {
			if (i.x < j.x or i.x == j.x and i.y < j.y) {
				for(auto k : step(4)) {
					for (int l : step(4)) {
						scorearray << (P{ i,j,k,l,score_calc(i,k,j,l) });
					}
				}
			}
		}
	}
	scorearray.sort_by([](const auto& a, const auto& b) {return a.score < b.score; });
	list<P> scorelist(scorearray.begin(), scorearray.end());

	{	//textfile output
		TextWriter writer(U"Banana.txt");
		for (const auto& i : scorelist)
			writer.writeln(U"{},{},{},{},{}"_fmt(i.p1, i.p2, i.r1, i.r2, i.score));
	}

	s3d::Array<shared_ptr<Graph>> nodes;
	Grid<weak_ptr<Graph>> graph;
	for (const auto& [p1, p2, d1, d2, score] : scorelist) {
		auto&& i1 = *find_if(graph.begin(), graph.end(), [&](const shared_ptr<Graph>& node) {return node->p == p1; });
		auto&& i2 = *find_if(graph.begin(), graph.end(), [&](const shared_ptr<Graph>& node) {return node->p == p2; });

	}

	Image im(image.size());
	DynamicTexture t(im);
	while (System::Update())
	{
		const auto rotate1 = [](const Image& image, int r) {
				switch (r) {
				case 0:
					return image.rotated180();
				case 1:
					return image.rotated90();
				case 2:
					return image;
				case 3:
					return image.rotated270();
				}
			};
		const auto rotate2 = [](const Image& image, int r) {
				switch (r) {
				case 0:
					return image;
				case 1:
					return image.rotated270();
				case 2:
					return image.rotated180();
				case 3:
					return image.rotated90();
				}
			};
		for (auto i : step(Size(16, 5))) {
			rotate1(tiles[scorearray[i.x + i.y * 16].p1], scorearray[i.x + i.y * 16].r1).overwrite(im, { tile_size.x * i.x,tile_size.y * i.y * 2 });
			rotate2(tiles[scorearray[i.x + i.y * 16].p2], scorearray[i.x + i.y * 16].r2).overwrite(im, { tile_size.x * i.x,tile_size.y * (i.y * 2 + 1) });
		}
		t.fill(im);
		t.draw();
	}
}
