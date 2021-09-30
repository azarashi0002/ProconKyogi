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
pair<Point, int> GetP2PosAndRotate(const Point& p1, int p1r, int p1e, int p2e) {
	return make_pair<Point, int>(p1 + s3d::Array<Point>{ Point::Up(), Point::Right(), Point::Down(), Point::Left() }[(p1r + p1e) % 4], (p1r + p1e + p2e + 2) % 4);
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
	Point suffledpos;
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

	struct ScoreData {
		Point p1, p2;
		int e1, e2;
		double score;
	};
	const auto score_calc = [&](const Point& p1, int d1, const Point& p2, int d2) {
		return (edge[p1][d1] - edge[p2][d2].colwise().reverse()).rowwise().squaredNorm().sum();
	};
	s3d::Array<ScoreData> scorearray;
	for (auto i : step(tiles.size())) {
		for (auto j : step(tiles.size())) {
			if (i.x < j.x or i.x == j.x and i.y < j.y) {
				for(auto k : step(4)) {
					for (int l : step(4)) {
						scorearray << (ScoreData{ i,j,k,l,score_calc(i,k,j,l) });
					}
				}
			}
		}
	}
	scorearray.sort_by([](const auto& a, const auto& b) {return a.score < b.score; });
	list<ScoreData> scorelist(scorearray.begin(), scorearray.end());

	{	//textfile output
		TextWriter writer(U"Banana.txt");
		for (const auto& i : scorelist)
			writer.writeln(U"{},{},{},{},{}"_fmt(i.p1, i.p2, i.e1, i.e2, i.score));
	}

	Grid<Optional<Graph>> graph(32, 32);	//operator[]にはグラフ座標
	Grid<Point> TilePToGraphP(tiles.size());
	s3d::Array<Point> used;	//位置が決定したタイル座標
	graph[{15, 15}] = Graph{ scorelist.front().p1,0 };
	TilePToGraphP[scorelist.front().p1] = Point(15, 15);
	used << scorelist.front().p1;

	const auto updategraph = [&]() {
		Optional<s3d::Array<ScoreData>::iterator> minscore;
		for (const auto& i : used) {
			Logger << U"タイル{},グラフ{}の周りを探します"_fmt(i, TilePToGraphP[i]);
			for (auto j = scorearray.begin(); j != scorearray.end(); ++j) {
				const auto& [p1, p2, e1, e2, score] = *j;
				if (not (p1 == i or p2 == i)) continue;
				const Point t = (p1 == i) ? p1 : p2;
				const Point target = (p1 == i) ? p2 : p1;
				if (used.includes(target)) continue;
				if (not minscore or score < (*minscore)->score) {
					const auto [p, r] = GetP2PosAndRotate(TilePToGraphP[t], graph[TilePToGraphP[t]]->r, e1, e2);
					if (graph[p])continue;
					minscore = j;
				}
			}
		}
		const Point p1 = (used.includes((*minscore)->p1)) ? (*minscore)->p1 : (*minscore)->p2;
		const int e1 = (used.includes((*minscore)->p1)) ? (*minscore)->e1 : (*minscore)->e2;
		const Point p2 = (used.includes((*minscore)->p1)) ? (*minscore)->p2 : (*minscore)->p1;
		const int e2 = (used.includes((*minscore)->p1)) ? (*minscore)->e2 : (*minscore)->e1;
		const auto [p, r] = GetP2PosAndRotate(TilePToGraphP[p1], graph[TilePToGraphP[p1]]->r, e1, e2);
		
		graph[p] = Graph{ p2,r };
		TilePToGraphP[p2] = p;
		used << p2;
	};


	for ([[maybe_unused]] int _ : step(/**15/*/tiles.num_elements() - 1/**/)) {
		updategraph();
	}

	Image im(tile_size * graph.size());
	DynamicTexture t(im);
	while (System::Update())
	{
		const auto rotate= [](const Image& image, int r) {
			switch (r) {
			case 0:
				return image;
			case 1:
				return image.rotated90();
			case 2:
				return image.rotated180();
			case 3:
				return image.rotated270();
			}
		};
		for (const auto& i : step(graph.size())) {
			if (const auto& n = graph[i]; n.has_value()) {
				rotate(tiles[n->suffledpos], n->r).overwrite(im, tile_size* i);
			}
		}
		t.fill(im);
		t.scaled(0.3).draw();
		//updategraph();
	}
}
