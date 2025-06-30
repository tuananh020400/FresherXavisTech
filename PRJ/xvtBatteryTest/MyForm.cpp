#include "MyForm.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <xvtCV/ColorDefine.h>
#include <xvtCV/xvtTypes.h>
#include <xvtCV/Utils.h>
#include "xvtCV/xvtCTAngleSelect.h"
using namespace System::IO;

#define INI_SECTION_LOWER L"INI_SECTION_LOWER"
#define INI_SECTION_UPPER L"INI_SECTION_UPPER"

using namespace xvt;

std::wstring ToWString(String^ s) {
	// Marshal managed string to unmanaged
	const wchar_t* chars = (const wchar_t*)(System::Runtime::InteropServices::Marshal::StringToHGlobalUni(s)).ToPointer();
	std::wstring wstr(chars);

	// Free the unmanaged memory
	System::Runtime::InteropServices::Marshal::FreeHGlobal(System::IntPtr((void*)chars));

	return wstr;
}

VecString vtFileNames;
using MSGBox = System::Windows::Forms::MessageBox;
namespace xvtBatteryTest
{

cv::Mat StretchImage(const cv::Mat& img, double c_min, double c_max)
{
	cv::Mat result(img.size(), CV_16UC1);
	auto alpha = 65535.0f / (c_max - c_min);
	auto beta = -c_min * alpha;
	img.convertTo(result, result.type(), alpha, beta);
	/*for (int y = 0; y < img.rows; y++)
	{
		for (int x = 0; x < img.cols; x++)
		{
			float pixelValue = img.at<float>(y, x);

            float normalizedValue = pixelValue * alpha + beta;
			uint16_t outputValue = cv::saturate_cast<uint16_t>(normalizedValue);

			result.at<uint16_t>(y, x) = outputValue;
		}
	}*/
	return result;
}

String^ GetFile(String^ filter=nullptr, String^ title=nullptr)
{
	String^ fileName = nullptr;
	OpenFileDialog^ openFileDialog1 = gcnew OpenFileDialog;
	//openFileDialog1->InitialDirectory = System::IO::Directory::GetCurrentDirectory();
	openFileDialog1->FilterIndex = 2;
	if (!String::IsNullOrWhiteSpace(title))
	{
		openFileDialog1->Title = title;
	}

	if(filter != nullptr)
		openFileDialog1->Filter = filter;

	if (openFileDialog1->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		IO::Stream^ myStream = openFileDialog1->OpenFile();
		if (myStream != nullptr)
		{
			fileName = gcnew String(openFileDialog1->FileName);
			myStream->Close();
		}
	}

	return fileName;
}

array<String^>^ GetFiles(String^ directory, String^ filter, bool recursive)
{
	array<String^>^ fileList = nullptr;

	if (String::IsNullOrWhiteSpace(directory)) return fileList;

	// Specify search option based on whether recursive search is desired
	SearchOption searchOption = recursive ? SearchOption::AllDirectories : SearchOption::TopDirectoryOnly;

	try
	{
		if (String::IsNullOrWhiteSpace(filter)) filter = gcnew String("*.*");
		// Get all files matching the filter within the directory
		fileList = Directory::GetFiles(directory, filter, searchOption);
	}
	catch (UnauthorizedAccessException^ e)
	{
		Console::WriteLine("Access denied to directory: " + directory);
		Console::WriteLine("Exception: " + e->Message);
	}
	catch (Exception^ e)
	{
		Console::WriteLine("Error accessing directory: " + directory);
		Console::WriteLine("Exception: " + e->Message);
	}

	return fileList;
}

String^ GetFolder()
{
	System::String^ path = nullptr;
	FolderBrowserDialog^ folderDialog = gcnew FolderBrowserDialog();
	folderDialog->Description = "Select a folder";
	folderDialog->ShowNewFolderButton = true;

	//folderDialog->SelectedPath = "";

	if (folderDialog->ShowDialog() == System::Windows::Forms::DialogResult::OK)
	{
		path = gcnew String(folderDialog->SelectedPath);
	}
	return path;
}

Void SetRowHeader(DataGridView^ gridView)
{
	if (!gridView) return;
    for (int i = 0; i < gridView->Rows->Count; i++)
	{
		auto row = gridView->Rows[i];
		row->HeaderCell->Value = (row->Index + 1).ToString();
	}
}

MyForm::~MyForm()
{
	if (components)
	{
		delete components;
	}

	if (mIspLower) delete mIspLower;
	if (mIspUpper) delete mIspUpper;
}

Void MyForm::LoadRecipe()
{
	if (!mRecipePath)
	{
		MSGBox::Show("Recipe path is invalid!");
		return;
	}

	bool rtn = false;
	auto recipePath = ToWString(mRecipePath);

	xvt::CTPropeties lowerLoader(*mIspLower);
	rtn = lowerLoader.LoadINI(recipePath, INI_SECTION_LOWER);
	if (!rtn)
	{
		MSGBox::Show("Lower: Some parameters are not load correctly!");
	}

	xvt::CTPropeties upperLoader(*mIspUpper);
	rtn = upperLoader.LoadINI(recipePath, INI_SECTION_UPPER);
	if (!rtn)
	{
		MSGBox::Show("Upper: Some parameters are not load correctly!");
	}
	progressBarInspect->Value = 0;
}

Void MyForm::GenerateSlices(String^ path, int start, int end, int step, int blurSize, double maxIten)
{
	if (String::IsNullOrWhiteSpace(path)) return;

	auto wPath = ToWString(path);
	auto ext = xvt::GetFileExtension(wPath);
	auto folder = xvt::GetParentDirectory(wPath);

	if (ext != L".raw") return;

	UpdateToolStripStatus("Start generate slice", 0);

	auto vol = xvt::ReadImageRaw(wPath, 256, 1024, 1024, CV_32FC1);

	if (vol.empty())return;

	cv::Mat inImg = vol[vol.size() / 2];
	auto imCenter = cv::Point2f(inImg.size().width / 2, inImg.size().height / 2);
	int mRadius = 512;

    float progressStep = start + step > end ? 100.0 : double(step) / (end - start) * 100.0;
	float progress = 0.f;
	for (auto i = start; i < end; i += step)
	{
		float lineAngle = i;

		auto slideAvgParrallel = xvt::ct::GetSliceMPR(vol,
													  imCenter,
													  lineAngle,
													  mRadius,
													  blurSize,
													  xvt::ct::SliceBlurType::ParrallelLine,
													  1,
													  false
		);
		cv::Mat im = StretchImage(slideAvgParrallel, 0, maxIten);
		cv::Mat padd = cv::Mat::zeros(im.rows * 3, im.cols, im.type());
		im.copyTo(padd(cv::Rect(0, im.rows, im.cols, im.rows)));

		std::wstring resName = folder + L"/slice_parrallel_" + xvt::ToWString(xvt::ToString(lineAngle, 2)) + L".tif";
		xvt::WriteImage(resName, padd);
		mDataTable->Rows->Add(gcnew String(resName.c_str()));

		progress += progressStep;
		UpdateToolStripStatus("Finish generate slice", progress);
	}
	UpdateToolStripStatus("Finish generate slice", 100);

}

Void MyForm::Inspect()
{
	auto rows = mDataTable->Rows;
	if (rows->Count < 1) return;

	std::vector<std::wstring> imgPaths(rows->Count);
	for (int i = 0; i < rows->Count; i++)
	{
        imgPaths[i] = ToWString((String^)rows[i]->ItemArray[0]);
	}

	float progressStep = 1.f / imgPaths.size()*100;
	float progress = 0.f;
	for (auto& path : imgPaths)
	{
		cv::Mat img = xvt::ReadImage(path, cv::ImreadModes::IMREAD_ANYDEPTH);
		auto folder = xvt::GetParentDirectory(path);
		auto imgName = xvt::GetFileNameWithoutExtension(path);
		auto resName = folder + L"/" + imgName + L"_res.png";

		String^ fileName = gcnew String(imgName.c_str());
		xvt::battery::ERR_CODE result = xvt::battery::ERR_CODE::NA;
		std::shared_ptr<xvt::IInspectionResult> IResult = nullptr;
		if (mIsUpperPart)
		{
			IResult = std::move(mIspUpper->Inspect(img));
		}
		else
		{
			IResult = std::move(mIspLower->Inspect(img));
		}

		String^ resStr = "Null";
		if(IResult)
		{
			IResult->DrawResult(img);
			xvt::WriteImage(resName, img);
			resStr = gcnew String(xvt::ToString(IResult->GetResult()).c_str());
		}
		progress += progressStep;
		UpdateToolStripStatus(fileName + " " + resStr, progress);
	}

	MSGBox::Show("Finish Inspect!");
}

Void MyForm::UpdateToolStripStatus(String^ msg, int progress)
{
    if (progress >= 0)
        progressBarInspect->Value = progress;
	toolStripStatusLabel1->Text = msg;
	statusStrip1->Update();
}

Void MyForm::btnRecipePath_Click(System::Object^ sender, System::EventArgs^ e)
{
	String^ filter = "INI Files|*.ini";
	auto path = GetFile(filter,"Select Recipe");
	if (path)
	{
		txtRecipePath->Text = path;
		mRecipePath = path;
	}
}

Void MyForm::btnRecipeLoad_Click(System::Object^ sender, System::EventArgs^ e)
{
	LoadRecipe();
	MSGBox::Show("Load Complete!");
}

Void MyForm::btnOpenImage_Click(System::Object^ sender, System::EventArgs^ e)
{
	String^ imgFilter = "Image Files|*.tif;*.bmp;*.jpg;*.jpeg;*.png;*.tiff";
	auto path = GetFile(imgFilter, "Select Image");
	if (path)
	{
		mDataTable->Rows->Add(path);
	}
	progressBarInspect->Value = 0;
}

Void MyForm::dataGridListFiles_RowsAdded(System::Object^ sender, System::Windows::Forms::DataGridViewRowsAddedEventArgs^ e)
{
	SetRowHeader(dataGridListFiles);
}

Void MyForm::dataGridListFiles_RowsRemoved(System::Object^ sender, System::Windows::Forms::DataGridViewRowsRemovedEventArgs^ e)
{
	SetRowHeader(dataGridListFiles);
}

Void MyForm::btnFolderPath_Click(System::Object^ sender, System::EventArgs^ e)
{
	auto path = GetFolder();
	if (path != nullptr)
	{
		txtFolderPath->Text = path;
		mFolderPath = path;
	}
}

Void MyForm::btnFolderLoad_Click(System::Object^ sender, System::EventArgs^ e)
{
	auto imglist = GetFiles(mFolderPath, mFileFilter, mIsRecursiveSearch);
	if (imglist == nullptr) return;
	for (int i = 0; i < imglist->Length; i++)
	{
		mDataTable->Rows->Add(imglist[i]);
	}
	progressBarInspect->Value = 0;
}

Void MyForm::btnClearGridView_Click(System::Object^ sender, System::EventArgs^ e)
{
	mDataTable->Clear();
}

Void MyForm::btnGenerateSlice_Click(System::Object^ sender, System::EventArgs^ e)
{
	auto path	 = txtVolume->Text;
	int start	 = int(numAngleStart->Value);
	int end		 = int(numAngleEnd->Value);
	int step	 = int(numAngleStep->Value);
	int blurSize = int(numBlurSize->Value);
	double maxInten = double(numMax->Value);
	if (start > end) std::swap(start, end);
	GenerateSlices(path, start, end, step, blurSize, maxInten);

	return Void();
}

Void MyForm::txtFolderPath_TextChanged(System::Object^ sender, System::EventArgs^ e)
{
	String^ txt = txtFolderPath->Text;
	if (String::IsNullOrWhiteSpace(txt)) return;
	mFolderPath = txt;
}

Void MyForm::txtImageFilter_TextChanged(System::Object^ sender, System::EventArgs^ e)
{
	String^ txt = txtImageFilter->Text;
	if (String::IsNullOrWhiteSpace(txt)) return;
	mFileFilter = txt;
}

Void MyForm::txtRecipePath_TextChanged(System::Object^ sender, System::EventArgs^ e)
{
	String^ txt = txtRecipePath->Text;
	if (String::IsNullOrWhiteSpace(txt)) return;
	mRecipePath = txt;
}

Void MyForm::btnStart_Click(System::Object^ sender, System::EventArgs^ e)
{
	progressBarInspect->Value = 0;
	LoadRecipe();
	Inspect();
}

Void MyForm::btnExit_Click(System::Object^ sender, System::EventArgs^ e)
{
	this->Close();
}

Void MyForm::cBoxBatteryPart_SelectedIndexChanged(System::Object^ sender, System::EventArgs^ e)
{
	auto txt = cBoxBatteryPart->SelectedItem;
	mIsUpperPart = txt == "UPPER";
}

Void MyForm::checkboxFolderRecursive_CheckedChanged(System::Object^ sender, System::EventArgs^ e)
{
	mIsRecursiveSearch = checkboxFolderRecursive->Checked;
}

Void MyForm::buttonSaveBrowser_Click(System::Object^ sender, System::EventArgs^ e)
{
	auto path = GetFolder();
	if (path != nullptr)
	{
		txtSavePath->Text = path;
		mSaveFolder = path;
	}
}

Void MyForm::tBoxSavePath_TextChanged(System::Object^ sender, System::EventArgs^ e)
{
	String^ txt = txtSavePath->Text;
	if (String::IsNullOrWhiteSpace(txt)) return;
	mSaveFolder = txt;
}

void MyForm::InitializeComponent(void)
{
	this->txtFolderPath = (gcnew System::Windows::Forms::TextBox());
	this->btnOpenImage = (gcnew System::Windows::Forms::Button());
	this->panel1 = (gcnew System::Windows::Forms::Panel());
	this->panel2 = (gcnew System::Windows::Forms::Panel());
	this->gBoxLoadImages = (gcnew System::Windows::Forms::GroupBox());
	this->btnClearGridView = (gcnew System::Windows::Forms::Button());
	this->checkboxFolderRecursive = (gcnew System::Windows::Forms::CheckBox());
	this->btnFolderLoad = (gcnew System::Windows::Forms::Button());
	this->rButtonFolder = (gcnew System::Windows::Forms::RadioButton());
	this->rButtonFile = (gcnew System::Windows::Forms::RadioButton());
	this->txtImageFilter = (gcnew System::Windows::Forms::TextBox());
	this->labelFilter = (gcnew System::Windows::Forms::Label());
	this->btnFolderPath = (gcnew System::Windows::Forms::Button());
	this->dataGridListFiles = (gcnew System::Windows::Forms::DataGridView());
	this->gBoxSetting = (gcnew System::Windows::Forms::GroupBox());
	this->btnRecipePath = (gcnew System::Windows::Forms::Button());
	this->txtRecipePath = (gcnew System::Windows::Forms::TextBox());
	this->btnRecipeLoad = (gcnew System::Windows::Forms::Button());
	this->cBoxBatteryPart = (gcnew System::Windows::Forms::ComboBox());
	this->labelRecipePath = (gcnew System::Windows::Forms::Label());
	this->labelBatteryPart = (gcnew System::Windows::Forms::Label());
	this->btnStart = (gcnew System::Windows::Forms::Button());
	this->btnExit = (gcnew System::Windows::Forms::Button());
	this->gBoxSaveImages = (gcnew System::Windows::Forms::GroupBox());
	this->buttonSaveBrowser = (gcnew System::Windows::Forms::Button());
	this->txtSavePath = (gcnew System::Windows::Forms::TextBox());
	this->label5 = (gcnew System::Windows::Forms::Label());
	this->tBoxSuffix = (gcnew System::Windows::Forms::TextBox());
	this->labelSuffix = (gcnew System::Windows::Forms::Label());
	this->cBoxFormat = (gcnew System::Windows::Forms::ComboBox());
	this->labelFormat = (gcnew System::Windows::Forms::Label());
	this->gBoxInspect = (gcnew System::Windows::Forms::GroupBox());
	this->progressBarInspect = (gcnew System::Windows::Forms::ProgressBar());
	this->comboBox1 = (gcnew System::Windows::Forms::ComboBox());
	this->label4 = (gcnew System::Windows::Forms::Label());
	this->labelAbout = (gcnew System::Windows::Forms::Label());
	this->toolStripContainer1 = (gcnew System::Windows::Forms::ToolStripContainer());
	this->statusStrip1 = (gcnew System::Windows::Forms::StatusStrip());
	this->toolStripStatusLabel1 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
	this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
	this->cbxAvgType = (gcnew System::Windows::Forms::ComboBox());
	this->txtVolume = (gcnew System::Windows::Forms::TextBox());
	this->label7 = (gcnew System::Windows::Forms::Label());
	this->lblVolume = (gcnew System::Windows::Forms::Label());
	this->numAngleStep = (gcnew System::Windows::Forms::NumericUpDown());
	this->label1 = (gcnew System::Windows::Forms::Label());
	this->label3 = (gcnew System::Windows::Forms::Label());
	this->label6 = (gcnew System::Windows::Forms::Label());
	this->numAngleEnd = (gcnew System::Windows::Forms::NumericUpDown());
	this->btnGenerateSlice = (gcnew System::Windows::Forms::Button());
	this->label2 = (gcnew System::Windows::Forms::Label());
	this->numAngleStart = (gcnew System::Windows::Forms::NumericUpDown());
	this->numBlurSize = (gcnew System::Windows::Forms::NumericUpDown());
	this->numMax = (gcnew System::Windows::Forms::NumericUpDown());
	this->label8 = (gcnew System::Windows::Forms::Label());
	this->gBoxLoadImages->SuspendLayout();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridListFiles))->BeginInit();
	this->gBoxSetting->SuspendLayout();
	this->gBoxSaveImages->SuspendLayout();
	this->gBoxInspect->SuspendLayout();
	this->toolStripContainer1->BottomToolStripPanel->SuspendLayout();
	this->toolStripContainer1->ContentPanel->SuspendLayout();
	this->toolStripContainer1->SuspendLayout();
	this->statusStrip1->SuspendLayout();
	this->groupBox1->SuspendLayout();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleStep))->BeginInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleEnd))->BeginInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleStart))->BeginInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numBlurSize))->BeginInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numMax))->BeginInit();
	this->SuspendLayout();
	// 
	// txtFolderPath
	// 
	this->txtFolderPath->Location = System::Drawing::Point(108, 52);
	this->txtFolderPath->Margin = System::Windows::Forms::Padding(4);
	this->txtFolderPath->Name = L"txtFolderPath";
	this->txtFolderPath->Size = System::Drawing::Size(215, 23);
	this->txtFolderPath->TabIndex = 2;
	this->txtFolderPath->TextChanged += gcnew System::EventHandler(this, &MyForm::txtFolderPath_TextChanged);
	// 
	// btnOpenImage
	// 
	this->btnOpenImage->FlatStyle = System::Windows::Forms::FlatStyle::System;
	this->btnOpenImage->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
															static_cast<System::Byte>(0)));
	this->btnOpenImage->Location = System::Drawing::Point(108, 20);
	this->btnOpenImage->Margin = System::Windows::Forms::Padding(4);
	this->btnOpenImage->Name = L"btnOpenImage";
	this->btnOpenImage->Size = System::Drawing::Size(81, 27);
	this->btnOpenImage->TabIndex = 7;
	this->btnOpenImage->Text = L"Open";
	this->btnOpenImage->UseVisualStyleBackColor = false;
	this->btnOpenImage->Click += gcnew System::EventHandler(this, &MyForm::btnOpenImage_Click);
	// 
	// panel1
	// 
	this->panel1->Location = System::Drawing::Point(0, 0);
	this->panel1->Name = L"panel1";
	this->panel1->Size = System::Drawing::Size(200, 100);
	this->panel1->TabIndex = 0;
	// 
	// panel2
	// 
	this->panel2->Location = System::Drawing::Point(0, 0);
	this->panel2->Name = L"panel2";
	this->panel2->Size = System::Drawing::Size(200, 100);
	this->panel2->TabIndex = 0;
	// 
	// gBoxLoadImages
	// 
	this->gBoxLoadImages->Controls->Add(this->btnClearGridView);
	this->gBoxLoadImages->Controls->Add(this->checkboxFolderRecursive);
	this->gBoxLoadImages->Controls->Add(this->btnFolderLoad);
	this->gBoxLoadImages->Controls->Add(this->rButtonFolder);
	this->gBoxLoadImages->Controls->Add(this->rButtonFile);
	this->gBoxLoadImages->Controls->Add(this->txtImageFilter);
	this->gBoxLoadImages->Controls->Add(this->labelFilter);
	this->gBoxLoadImages->Controls->Add(this->btnFolderPath);
	this->gBoxLoadImages->Controls->Add(this->btnOpenImage);
	this->gBoxLoadImages->Controls->Add(this->txtFolderPath);
	this->gBoxLoadImages->Location = System::Drawing::Point(14, 109);
	this->gBoxLoadImages->Margin = System::Windows::Forms::Padding(4);
	this->gBoxLoadImages->Name = L"gBoxLoadImages";
	this->gBoxLoadImages->Padding = System::Windows::Forms::Padding(4);
	this->gBoxLoadImages->Size = System::Drawing::Size(446, 133);
	this->gBoxLoadImages->TabIndex = 23;
	this->gBoxLoadImages->TabStop = false;
	this->gBoxLoadImages->Text = L"Load Images";
	// 
	// btnClearGridView
	// 
	this->btnClearGridView->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold));
	this->btnClearGridView->Location = System::Drawing::Point(364, 18);
	this->btnClearGridView->Name = L"btnClearGridView";
	this->btnClearGridView->Size = System::Drawing::Size(73, 27);
	this->btnClearGridView->TabIndex = 30;
	this->btnClearGridView->Text = L"Clear";
	this->btnClearGridView->UseVisualStyleBackColor = true;
	this->btnClearGridView->Click += gcnew System::EventHandler(this, &MyForm::btnClearGridView_Click);
	// 
	// checkboxFolderRecursive
	// 
	this->checkboxFolderRecursive->AutoSize = true;
	this->checkboxFolderRecursive->Location = System::Drawing::Point(258, 89);
	this->checkboxFolderRecursive->Name = L"checkboxFolderRecursive";
	this->checkboxFolderRecursive->Size = System::Drawing::Size(73, 20);
	this->checkboxFolderRecursive->TabIndex = 29;
	this->checkboxFolderRecursive->Text = L"SubDirs";
	this->checkboxFolderRecursive->UseVisualStyleBackColor = true;
	this->checkboxFolderRecursive->CheckedChanged += gcnew System::EventHandler(this, &MyForm::checkboxFolderRecursive_CheckedChanged);
	// 
	// btnFolderLoad
	// 
	this->btnFolderLoad->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
															 static_cast<System::Byte>(0)));
	this->btnFolderLoad->Location = System::Drawing::Point(362, 52);
	this->btnFolderLoad->Margin = System::Windows::Forms::Padding(4);
	this->btnFolderLoad->Name = L"btnFolderLoad";
	this->btnFolderLoad->Size = System::Drawing::Size(74, 61);
	this->btnFolderLoad->TabIndex = 26;
	this->btnFolderLoad->Text = L"Load";
	this->btnFolderLoad->UseVisualStyleBackColor = false;
	this->btnFolderLoad->Click += gcnew System::EventHandler(this, &MyForm::btnFolderLoad_Click);
	// 
	// rButtonFolder
	// 
	this->rButtonFolder->AutoSize = true;
	this->rButtonFolder->Location = System::Drawing::Point(14, 55);
	this->rButtonFolder->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
	this->rButtonFolder->Name = L"rButtonFolder";
	this->rButtonFolder->Size = System::Drawing::Size(65, 20);
	this->rButtonFolder->TabIndex = 27;
	this->rButtonFolder->TabStop = true;
	this->rButtonFolder->Text = L"Folder:";
	this->rButtonFolder->UseVisualStyleBackColor = true;
	// 
	// rButtonFile
	// 
	this->rButtonFile->AutoSize = true;
	this->rButtonFile->Checked = true;
	this->rButtonFile->Location = System::Drawing::Point(14, 23);
	this->rButtonFile->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
	this->rButtonFile->Name = L"rButtonFile";
	this->rButtonFile->Size = System::Drawing::Size(49, 20);
	this->rButtonFile->TabIndex = 26;
	this->rButtonFile->TabStop = true;
	this->rButtonFile->Text = L"File:";
	this->rButtonFile->UseVisualStyleBackColor = true;
	this->rButtonFile->CheckedChanged += gcnew System::EventHandler(this, &MyForm::radioButton1_CheckedChanged);
	// 
	// txtImageFilter
	// 
	this->txtImageFilter->Location = System::Drawing::Point(108, 88);
	this->txtImageFilter->Margin = System::Windows::Forms::Padding(4);
	this->txtImageFilter->Name = L"txtImageFilter";
	this->txtImageFilter->Size = System::Drawing::Size(126, 23);
	this->txtImageFilter->TabIndex = 23;
	this->txtImageFilter->TextChanged += gcnew System::EventHandler(this, &MyForm::txtImageFilter_TextChanged);
	// 
	// labelFilter
	// 
	this->labelFilter->AutoSize = true;
	this->labelFilter->Location = System::Drawing::Point(33, 91);
	this->labelFilter->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelFilter->Name = L"labelFilter";
	this->labelFilter->Size = System::Drawing::Size(40, 16);
	this->labelFilter->TabIndex = 22;
	this->labelFilter->Text = L"Filter:";
	// 
	// btnFolderPath
	// 
	this->btnFolderPath->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
															 static_cast<System::Byte>(0)));
	this->btnFolderPath->Location = System::Drawing::Point(328, 51);
	this->btnFolderPath->Margin = System::Windows::Forms::Padding(4);
	this->btnFolderPath->Name = L"btnFolderPath";
	this->btnFolderPath->Size = System::Drawing::Size(33, 27);
	this->btnFolderPath->TabIndex = 21;
	this->btnFolderPath->Text = L"...";
	this->btnFolderPath->UseVisualStyleBackColor = false;
	this->btnFolderPath->Click += gcnew System::EventHandler(this, &MyForm::btnFolderPath_Click);
	// 
	// dataGridListFiles
	// 
	this->dataGridListFiles->AllowUserToAddRows = false;
	this->dataGridListFiles->AutoSizeColumnsMode = System::Windows::Forms::DataGridViewAutoSizeColumnsMode::Fill;
	this->dataGridListFiles->ColumnHeadersHeightSizeMode = System::Windows::Forms::DataGridViewColumnHeadersHeightSizeMode::AutoSize;
	this->dataGridListFiles->Location = System::Drawing::Point(472, 19);
	this->dataGridListFiles->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
	this->dataGridListFiles->Name = L"dataGridListFiles";
	this->dataGridListFiles->ReadOnly = true;
	this->dataGridListFiles->RowHeadersWidthSizeMode = System::Windows::Forms::DataGridViewRowHeadersWidthSizeMode::AutoSizeToDisplayedHeaders;
	this->dataGridListFiles->RowTemplate->Height = 24;
	this->dataGridListFiles->Size = System::Drawing::Size(425, 592);
	this->dataGridListFiles->TabIndex = 28;
	this->dataGridListFiles->RowsAdded += gcnew System::Windows::Forms::DataGridViewRowsAddedEventHandler(this, &MyForm::dataGridListFiles_RowsAdded);
	this->dataGridListFiles->RowsRemoved += gcnew System::Windows::Forms::DataGridViewRowsRemovedEventHandler(this, &MyForm::dataGridListFiles_RowsRemoved);
	// 
	// gBoxSetting
	// 
	this->gBoxSetting->Controls->Add(this->btnRecipePath);
	this->gBoxSetting->Controls->Add(this->txtRecipePath);
	this->gBoxSetting->Controls->Add(this->btnRecipeLoad);
	this->gBoxSetting->Controls->Add(this->cBoxBatteryPart);
	this->gBoxSetting->Controls->Add(this->labelRecipePath);
	this->gBoxSetting->Controls->Add(this->labelBatteryPart);
	this->gBoxSetting->Location = System::Drawing::Point(14, 13);
	this->gBoxSetting->Margin = System::Windows::Forms::Padding(4);
	this->gBoxSetting->Name = L"gBoxSetting";
	this->gBoxSetting->Padding = System::Windows::Forms::Padding(4);
	this->gBoxSetting->Size = System::Drawing::Size(446, 91);
	this->gBoxSetting->TabIndex = 24;
	this->gBoxSetting->TabStop = false;
	this->gBoxSetting->Text = L"Setting";
	// 
	// btnRecipePath
	// 
	this->btnRecipePath->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
															 static_cast<System::Byte>(0)));
	this->btnRecipePath->Location = System::Drawing::Point(328, 54);
	this->btnRecipePath->Margin = System::Windows::Forms::Padding(4);
	this->btnRecipePath->Name = L"btnRecipePath";
	this->btnRecipePath->Size = System::Drawing::Size(31, 27);
	this->btnRecipePath->TabIndex = 31;
	this->btnRecipePath->Text = L"...";
	this->btnRecipePath->UseVisualStyleBackColor = false;
	this->btnRecipePath->Click += gcnew System::EventHandler(this, &MyForm::btnRecipePath_Click);
	// 
	// txtRecipePath
	// 
	this->txtRecipePath->Location = System::Drawing::Point(108, 56);
	this->txtRecipePath->Margin = System::Windows::Forms::Padding(4);
	this->txtRecipePath->Name = L"txtRecipePath";
	this->txtRecipePath->Size = System::Drawing::Size(215, 23);
	this->txtRecipePath->TabIndex = 30;
	this->txtRecipePath->TextChanged += gcnew System::EventHandler(this, &MyForm::txtRecipePath_TextChanged);
	// 
	// btnRecipeLoad
	// 
	this->btnRecipeLoad->FlatStyle = System::Windows::Forms::FlatStyle::System;
	this->btnRecipeLoad->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
															 static_cast<System::Byte>(0)));
	this->btnRecipeLoad->Location = System::Drawing::Point(362, 18);
	this->btnRecipeLoad->Margin = System::Windows::Forms::Padding(4);
	this->btnRecipeLoad->Name = L"btnRecipeLoad";
	this->btnRecipeLoad->Size = System::Drawing::Size(75, 65);
	this->btnRecipeLoad->TabIndex = 29;
	this->btnRecipeLoad->Text = L"Load";
	this->btnRecipeLoad->UseVisualStyleBackColor = false;
	this->btnRecipeLoad->Click += gcnew System::EventHandler(this, &MyForm::btnRecipeLoad_Click);
	// 
	// cBoxBatteryPart
	// 
	this->cBoxBatteryPart->FormattingEnabled = true;
	this->cBoxBatteryPart->Items->AddRange(gcnew cli::array< System::Object^  >(3) { L"UPPER", L"LOWER", L"BOTH" });
	this->cBoxBatteryPart->Location = System::Drawing::Point(108, 24);
	this->cBoxBatteryPart->Margin = System::Windows::Forms::Padding(4);
	this->cBoxBatteryPart->Name = L"cBoxBatteryPart";
	this->cBoxBatteryPart->Size = System::Drawing::Size(91, 23);
	this->cBoxBatteryPart->TabIndex = 27;
	this->cBoxBatteryPart->SelectedIndexChanged += gcnew System::EventHandler(this, &MyForm::cBoxBatteryPart_SelectedIndexChanged);
	// 
	// labelRecipePath
	// 
	this->labelRecipePath->AutoSize = true;
	this->labelRecipePath->Location = System::Drawing::Point(8, 59);
	this->labelRecipePath->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelRecipePath->Name = L"labelRecipePath";
	this->labelRecipePath->Size = System::Drawing::Size(78, 16);
	this->labelRecipePath->TabIndex = 20;
	this->labelRecipePath->Text = L"Recipe Path:";
	// 
	// labelBatteryPart
	// 
	this->labelBatteryPart->AutoSize = true;
	this->labelBatteryPart->Location = System::Drawing::Point(8, 27);
	this->labelBatteryPart->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelBatteryPart->Name = L"labelBatteryPart";
	this->labelBatteryPart->Size = System::Drawing::Size(78, 16);
	this->labelBatteryPart->TabIndex = 14;
	this->labelBatteryPart->Text = L"Battery Part:";
	// 
	// btnStart
	// 
	this->btnStart->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
														static_cast<System::Byte>(0)));
	this->btnStart->Location = System::Drawing::Point(350, 621);
	this->btnStart->Margin = System::Windows::Forms::Padding(4);
	this->btnStart->Name = L"btnStart";
	this->btnStart->Size = System::Drawing::Size(110, 37);
	this->btnStart->TabIndex = 29;
	this->btnStart->Text = L"Start";
	this->btnStart->UseVisualStyleBackColor = false;
	this->btnStart->Click += gcnew System::EventHandler(this, &MyForm::btnStart_Click);
	// 
	// btnExit
	// 
	this->btnExit->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
													   static_cast<System::Byte>(0)));
	this->btnExit->Location = System::Drawing::Point(472, 621);
	this->btnExit->Margin = System::Windows::Forms::Padding(4);
	this->btnExit->Name = L"btnExit";
	this->btnExit->Size = System::Drawing::Size(110, 37);
	this->btnExit->TabIndex = 30;
	this->btnExit->Text = L"Exit";
	this->btnExit->UseVisualStyleBackColor = false;
	this->btnExit->Click += gcnew System::EventHandler(this, &MyForm::btnExit_Click);
	// 
	// gBoxSaveImages
	// 
	this->gBoxSaveImages->Controls->Add(this->buttonSaveBrowser);
	this->gBoxSaveImages->Controls->Add(this->txtSavePath);
	this->gBoxSaveImages->Controls->Add(this->label5);
	this->gBoxSaveImages->Controls->Add(this->tBoxSuffix);
	this->gBoxSaveImages->Controls->Add(this->labelSuffix);
	this->gBoxSaveImages->Controls->Add(this->cBoxFormat);
	this->gBoxSaveImages->Controls->Add(this->labelFormat);
	this->gBoxSaveImages->Location = System::Drawing::Point(14, 407);
	this->gBoxSaveImages->Margin = System::Windows::Forms::Padding(4);
	this->gBoxSaveImages->Name = L"gBoxSaveImages";
	this->gBoxSaveImages->Padding = System::Windows::Forms::Padding(4);
	this->gBoxSaveImages->Size = System::Drawing::Size(446, 142);
	this->gBoxSaveImages->TabIndex = 28;
	this->gBoxSaveImages->TabStop = false;
	this->gBoxSaveImages->Text = L"Save Images";
	// 
	// buttonSaveBrowser
	// 
	this->buttonSaveBrowser->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
																 static_cast<System::Byte>(0)));
	this->buttonSaveBrowser->Location = System::Drawing::Point(402, 99);
	this->buttonSaveBrowser->Margin = System::Windows::Forms::Padding(4);
	this->buttonSaveBrowser->Name = L"buttonSaveBrowser";
	this->buttonSaveBrowser->Size = System::Drawing::Size(36, 27);
	this->buttonSaveBrowser->TabIndex = 29;
	this->buttonSaveBrowser->Text = L"...";
	this->buttonSaveBrowser->UseVisualStyleBackColor = false;
	this->buttonSaveBrowser->Click += gcnew System::EventHandler(this, &MyForm::buttonSaveBrowser_Click);
	// 
	// txtSavePath
	// 
	this->txtSavePath->Location = System::Drawing::Point(109, 101);
	this->txtSavePath->Margin = System::Windows::Forms::Padding(4);
	this->txtSavePath->Name = L"txtSavePath";
	this->txtSavePath->Size = System::Drawing::Size(296, 23);
	this->txtSavePath->TabIndex = 29;
	this->txtSavePath->TextChanged += gcnew System::EventHandler(this, &MyForm::tBoxSavePath_TextChanged);
	// 
	// label5
	// 
	this->label5->AutoSize = true;
	this->label5->Location = System::Drawing::Point(7, 104);
	this->label5->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label5->Name = L"label5";
	this->label5->Size = System::Drawing::Size(53, 16);
	this->label5->TabIndex = 28;
	this->label5->Text = L"Save to:";
	// 
	// tBoxSuffix
	// 
	this->tBoxSuffix->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Left | System::Windows::Forms::AnchorStyles::Right));
	this->tBoxSuffix->Location = System::Drawing::Point(109, 28);
	this->tBoxSuffix->Margin = System::Windows::Forms::Padding(4);
	this->tBoxSuffix->MaxLength = 1000;
	this->tBoxSuffix->Name = L"tBoxSuffix";
	this->tBoxSuffix->Size = System::Drawing::Size(125, 23);
	this->tBoxSuffix->TabIndex = 29;
	this->tBoxSuffix->Text = L"-suffix";
	// 
	// labelSuffix
	// 
	this->labelSuffix->AutoSize = true;
	this->labelSuffix->Location = System::Drawing::Point(7, 32);
	this->labelSuffix->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelSuffix->Name = L"labelSuffix";
	this->labelSuffix->Size = System::Drawing::Size(46, 16);
	this->labelSuffix->TabIndex = 28;
	this->labelSuffix->Text = L"Suffix:";
	// 
	// cBoxFormat
	// 
	this->cBoxFormat->FormattingEnabled = true;
	this->cBoxFormat->Items->AddRange(gcnew cli::array< System::Object^  >(3) { L"JPG", L"PNG", L"TIF" });
	this->cBoxFormat->Location = System::Drawing::Point(109, 65);
	this->cBoxFormat->Margin = System::Windows::Forms::Padding(4);
	this->cBoxFormat->Name = L"cBoxFormat";
	this->cBoxFormat->Size = System::Drawing::Size(80, 23);
	this->cBoxFormat->TabIndex = 26;
	// 
	// labelFormat
	// 
	this->labelFormat->AutoSize = true;
	this->labelFormat->Location = System::Drawing::Point(7, 69);
	this->labelFormat->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelFormat->Name = L"labelFormat";
	this->labelFormat->Size = System::Drawing::Size(52, 16);
	this->labelFormat->TabIndex = 20;
	this->labelFormat->Text = L"Format:";
	// 
	// gBoxInspect
	// 
	this->gBoxInspect->Controls->Add(this->progressBarInspect);
	this->gBoxInspect->Controls->Add(this->comboBox1);
	this->gBoxInspect->Controls->Add(this->label4);
	this->gBoxInspect->Location = System::Drawing::Point(14, 548);
	this->gBoxInspect->Margin = System::Windows::Forms::Padding(4);
	this->gBoxInspect->Name = L"gBoxInspect";
	this->gBoxInspect->Padding = System::Windows::Forms::Padding(4);
	this->gBoxInspect->Size = System::Drawing::Size(446, 63);
	this->gBoxInspect->TabIndex = 29;
	this->gBoxInspect->TabStop = false;
	this->gBoxInspect->Text = L"Inspect";
	// 
	// progressBarInspect
	// 
	this->progressBarInspect->Location = System::Drawing::Point(14, 27);
	this->progressBarInspect->Margin = System::Windows::Forms::Padding(3, 4, 3, 4);
	this->progressBarInspect->Name = L"progressBarInspect";
	this->progressBarInspect->Size = System::Drawing::Size(425, 27);
	this->progressBarInspect->TabIndex = 29;
	// 
	// comboBox1
	// 
	this->comboBox1->FormattingEnabled = true;
	this->comboBox1->Location = System::Drawing::Point(108, 109);
	this->comboBox1->Margin = System::Windows::Forms::Padding(4);
	this->comboBox1->Name = L"comboBox1";
	this->comboBox1->Size = System::Drawing::Size(134, 23);
	this->comboBox1->TabIndex = 26;
	// 
	// label4
	// 
	this->label4->AutoSize = true;
	this->label4->Location = System::Drawing::Point(4, 112);
	this->label4->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label4->Name = L"label4";
	this->label4->Size = System::Drawing::Size(49, 16);
	this->label4->TabIndex = 20;
	this->label4->Text = L"Format";
	// 
	// labelAbout
	// 
	this->labelAbout->AutoSize = true;
	this->labelAbout->Location = System::Drawing::Point(19, 629);
	this->labelAbout->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->labelAbout->Name = L"labelAbout";
	this->labelAbout->Size = System::Drawing::Size(168, 16);
	this->labelAbout->TabIndex = 28;
	this->labelAbout->Text = L"Xavis Tech (xavistech.com)";
	// 
	// toolStripContainer1
	// 
	// 
	// toolStripContainer1.BottomToolStripPanel
	// 
	this->toolStripContainer1->BottomToolStripPanel->Controls->Add(this->statusStrip1);
	// 
	// toolStripContainer1.ContentPanel
	// 
	this->toolStripContainer1->ContentPanel->AutoScroll = true;
	this->toolStripContainer1->ContentPanel->Controls->Add(this->groupBox1);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->labelAbout);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->gBoxInspect);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->dataGridListFiles);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->gBoxSaveImages);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->btnExit);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->btnStart);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->gBoxSetting);
	this->toolStripContainer1->ContentPanel->Controls->Add(this->gBoxLoadImages);
	this->toolStripContainer1->ContentPanel->Size = System::Drawing::Size(905, 662);
	this->toolStripContainer1->Dock = System::Windows::Forms::DockStyle::Fill;
	this->toolStripContainer1->LeftToolStripPanelVisible = false;
	this->toolStripContainer1->Location = System::Drawing::Point(0, 0);
	this->toolStripContainer1->Name = L"toolStripContainer1";
	this->toolStripContainer1->RightToolStripPanelVisible = false;
	this->toolStripContainer1->Size = System::Drawing::Size(905, 684);
	this->toolStripContainer1->TabIndex = 31;
	this->toolStripContainer1->Text = L"toolStripContainer1";
	this->toolStripContainer1->TopToolStripPanelVisible = false;
	// 
	// statusStrip1
	// 
	this->statusStrip1->Dock = System::Windows::Forms::DockStyle::None;
	this->statusStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) { this->toolStripStatusLabel1 });
	this->statusStrip1->Location = System::Drawing::Point(0, 0);
	this->statusStrip1->Name = L"statusStrip1";
	this->statusStrip1->RenderMode = System::Windows::Forms::ToolStripRenderMode::Professional;
	this->statusStrip1->Size = System::Drawing::Size(905, 22);
	this->statusStrip1->TabIndex = 0;
	// 
	// toolStripStatusLabel1
	// 
	this->toolStripStatusLabel1->Name = L"toolStripStatusLabel1";
	this->toolStripStatusLabel1->Size = System::Drawing::Size(118, 17);
	this->toolStripStatusLabel1->Text = L"toolStripStatusLabel1";
	// 
	// groupBox1
	// 
	this->groupBox1->Controls->Add(this->cbxAvgType);
	this->groupBox1->Controls->Add(this->txtVolume);
	this->groupBox1->Controls->Add(this->label7);
	this->groupBox1->Controls->Add(this->lblVolume);
	this->groupBox1->Controls->Add(this->numAngleStep);
	this->groupBox1->Controls->Add(this->label1);
	this->groupBox1->Controls->Add(this->label3);
	this->groupBox1->Controls->Add(this->label8);
	this->groupBox1->Controls->Add(this->label6);
	this->groupBox1->Controls->Add(this->numAngleEnd);
	this->groupBox1->Controls->Add(this->btnGenerateSlice);
	this->groupBox1->Controls->Add(this->label2);
	this->groupBox1->Controls->Add(this->numAngleStart);
	this->groupBox1->Controls->Add(this->numMax);
	this->groupBox1->Controls->Add(this->numBlurSize);
	this->groupBox1->Location = System::Drawing::Point(14, 242);
	this->groupBox1->Name = L"groupBox1";
	this->groupBox1->Size = System::Drawing::Size(446, 157);
	this->groupBox1->TabIndex = 34;
	this->groupBox1->TabStop = false;
	this->groupBox1->Text = L"Volume";
	// 
	// cbxAvgType
	// 
	this->cbxAvgType->FormattingEnabled = true;
	this->cbxAvgType->Items->AddRange(gcnew cli::array< System::Object^  >(3) { L"UPPER", L"LOWER", L"BOTH" });
	this->cbxAvgType->Location = System::Drawing::Point(267, 63);
	this->cbxAvgType->Margin = System::Windows::Forms::Padding(4);
	this->cbxAvgType->Name = L"cbxAvgType";
	this->cbxAvgType->Size = System::Drawing::Size(87, 23);
	this->cbxAvgType->TabIndex = 33;
	// 
	// txtVolume
	// 
	this->txtVolume->Location = System::Drawing::Point(86, 29);
	this->txtVolume->Margin = System::Windows::Forms::Padding(4);
	this->txtVolume->Name = L"txtVolume";
	this->txtVolume->Size = System::Drawing::Size(268, 23);
	this->txtVolume->TabIndex = 23;
	// 
	// label7
	// 
	this->label7->AutoSize = true;
	this->label7->Location = System::Drawing::Point(202, 66);
	this->label7->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label7->Name = L"label7";
	this->label7->Size = System::Drawing::Size(66, 16);
	this->label7->TabIndex = 32;
	this->label7->Text = L"Avg. Type";
	// 
	// lblVolume
	// 
	this->lblVolume->AutoSize = true;
	this->lblVolume->Location = System::Drawing::Point(9, 32);
	this->lblVolume->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->lblVolume->Name = L"lblVolume";
	this->lblVolume->Size = System::Drawing::Size(49, 16);
	this->lblVolume->TabIndex = 22;
	this->lblVolume->Text = L"Volume";
	// 
	// numAngleStep
	// 
	this->numAngleStep->Location = System::Drawing::Point(85, 118);
	this->numAngleStep->Name = L"numAngleStep";
	this->numAngleStep->Size = System::Drawing::Size(88, 23);
	this->numAngleStep->TabIndex = 33;
	this->numAngleStep->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 10, 0, 0, 0 });
	// 
	// label1
	// 
	this->label1->AutoSize = true;
	this->label1->Location = System::Drawing::Point(9, 62);
	this->label1->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label1->Name = L"label1";
	this->label1->Size = System::Drawing::Size(70, 16);
	this->label1->TabIndex = 22;
	this->label1->Text = L"Start Angle";
	// 
	// label3
	// 
	this->label3->AutoSize = true;
	this->label3->Location = System::Drawing::Point(9, 118);
	this->label3->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label3->Name = L"label3";
	this->label3->Size = System::Drawing::Size(68, 16);
	this->label3->TabIndex = 32;
	this->label3->Text = L"Step Angle";
	// 
	// label6
	// 
	this->label6->AutoSize = true;
	this->label6->Location = System::Drawing::Point(202, 95);
	this->label6->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label6->Name = L"label6";
	this->label6->Size = System::Drawing::Size(58, 16);
	this->label6->TabIndex = 22;
	this->label6->Text = L"Blur Size";
	// 
	// numAngleEnd
	// 
	this->numAngleEnd->Location = System::Drawing::Point(85, 89);
	this->numAngleEnd->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 360, 0, 0, 0 });
	this->numAngleEnd->Name = L"numAngleEnd";
	this->numAngleEnd->Size = System::Drawing::Size(88, 23);
	this->numAngleEnd->TabIndex = 33;
	this->numAngleEnd->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 180, 0, 0, 0 });
	// 
	// btnGenerateSlice
	// 
	this->btnGenerateSlice->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 8.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
																static_cast<System::Byte>(0)));
	this->btnGenerateSlice->Location = System::Drawing::Point(362, 21);
	this->btnGenerateSlice->Margin = System::Windows::Forms::Padding(4);
	this->btnGenerateSlice->Name = L"btnGenerateSlice";
	this->btnGenerateSlice->Size = System::Drawing::Size(74, 121);
	this->btnGenerateSlice->TabIndex = 26;
	this->btnGenerateSlice->Text = L"Generate";
	this->btnGenerateSlice->UseVisualStyleBackColor = false;
	this->btnGenerateSlice->Click += gcnew System::EventHandler(this, &MyForm::btnGenerateSlice_Click);
	// 
	// label2
	// 
	this->label2->AutoSize = true;
	this->label2->Location = System::Drawing::Point(9, 91);
	this->label2->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label2->Name = L"label2";
	this->label2->Size = System::Drawing::Size(65, 16);
	this->label2->TabIndex = 32;
	this->label2->Text = L"End Angle";
	// 
	// numAngleStart
	// 
	this->numAngleStart->Location = System::Drawing::Point(86, 60);
	this->numAngleStart->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 180, 0, 0, 0 });
	this->numAngleStart->Name = L"numAngleStart";
	this->numAngleStart->Size = System::Drawing::Size(87, 23);
	this->numAngleStart->TabIndex = 31;
	// 
	// numBlurSize
	// 
	this->numBlurSize->Location = System::Drawing::Point(267, 91);
	this->numBlurSize->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 0 });
	this->numBlurSize->Name = L"numBlurSize";
	this->numBlurSize->Size = System::Drawing::Size(87, 23);
	this->numBlurSize->TabIndex = 31;
	this->numBlurSize->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 15, 0, 0, 0 });
	// 
	// numMax
	// 
	this->numMax->DecimalPlaces = 3;
	this->numMax->Increment = System::Decimal(gcnew cli::array< System::Int32 >(4) { 1, 0, 0, 131072 });
	this->numMax->Location = System::Drawing::Point(268, 119);
	this->numMax->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) { 2, 0, 0, 0 });
	this->numMax->Name = L"numMax";
	this->numMax->Size = System::Drawing::Size(87, 23);
	this->numMax->TabIndex = 31;
	this->numMax->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) { 95, 0, 0, 196608 });
	// 
	// label8
	// 
	this->label8->AutoSize = true;
	this->label8->Location = System::Drawing::Point(203, 123);
	this->label8->Margin = System::Windows::Forms::Padding(4, 0, 4, 0);
	this->label8->Name = L"label8";
	this->label8->Size = System::Drawing::Size(33, 16);
	this->label8->TabIndex = 22;
	this->label8->Text = L"Max";
	// 
	// MyForm
	// 
	this->AutoScaleDimensions = System::Drawing::SizeF(7, 15);
	this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
	this->ClientSize = System::Drawing::Size(905, 684);
	this->Controls->Add(this->toolStripContainer1);
	this->Font = (gcnew System::Drawing::Font(L"Times New Roman", 10, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
											  static_cast<System::Byte>(0)));
	this->FormBorderStyle = System::Windows::Forms::FormBorderStyle::Fixed3D;
	this->Margin = System::Windows::Forms::Padding(4);
	this->MaximizeBox = false;
	this->Name = L"MyForm";
	this->SizeGripStyle = System::Windows::Forms::SizeGripStyle::Hide;
	this->StartPosition = System::Windows::Forms::FormStartPosition::CenterScreen;
	this->Text = L"xvtBattery";
	this->gBoxLoadImages->ResumeLayout(false);
	this->gBoxLoadImages->PerformLayout();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->dataGridListFiles))->EndInit();
	this->gBoxSetting->ResumeLayout(false);
	this->gBoxSetting->PerformLayout();
	this->gBoxSaveImages->ResumeLayout(false);
	this->gBoxSaveImages->PerformLayout();
	this->gBoxInspect->ResumeLayout(false);
	this->gBoxInspect->PerformLayout();
	this->toolStripContainer1->BottomToolStripPanel->ResumeLayout(false);
	this->toolStripContainer1->BottomToolStripPanel->PerformLayout();
	this->toolStripContainer1->ContentPanel->ResumeLayout(false);
	this->toolStripContainer1->ContentPanel->PerformLayout();
	this->toolStripContainer1->ResumeLayout(false);
	this->toolStripContainer1->PerformLayout();
	this->statusStrip1->ResumeLayout(false);
	this->statusStrip1->PerformLayout();
	this->groupBox1->ResumeLayout(false);
	this->groupBox1->PerformLayout();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleStep))->EndInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleEnd))->EndInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numAngleStart))->EndInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numBlurSize))->EndInit();
	(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->numMax))->EndInit();
	this->ResumeLayout(false);

}
}
