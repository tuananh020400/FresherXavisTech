#pragma once
#include <iostream>

#include <xvtBattery/CTBatteryInspectorBase.h>
#include <xvtBattery/CylinderSetting.h>

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;

namespace xvtBatteryTest {
/// <summary>
/// Summary for MyForm
/// </summary>
public ref class MyForm : public Form
{
public:
	MyForm(void)
	{
		InitializeComponent();
		//
		//TODO: Add the constructor code here
		//
		
		mGridviewBindingSource->DataSource = mImgList;

		// Bind to DataGridView
		dataGridListFiles->DataSource = mDataTable;
		mDataTable->Columns->Add("File Name", String::typeid);

		cBoxBatteryPart->SelectedIndex = 0;
	}

protected:
	/// <summary>
	/// Clean up any resources being used.
	/// </summary>
	~MyForm();

#pragma region Windows Form Designer generated code
	/// <summary>
	/// Required method for Designer support - do not modify
	/// the contents of this method with the code editor.
	/// </summary>
	void InitializeComponent(void);
#pragma endregion

private:
	Void cBoxBatteryPart_SelectedIndexChanged(System::Object^ sender, System::EventArgs^ e);
	Void radioButton1_CheckedChanged(System::Object^ sender, System::EventArgs^ e) {}

	Void txtRecipePath_TextChanged(System::Object^ sender, System::EventArgs^ e);
	Void btnRecipePath_Click(System::Object^ sender, System::EventArgs^ e);
	Void btnRecipeLoad_Click(System::Object^ sender, System::EventArgs^ e);

	Void buttonSaveBrowser_Click(System::Object^ sender, System::EventArgs^ e);
	Void tBoxSavePath_TextChanged(System::Object^ sender, System::EventArgs^ e);

	Void dataGridListFiles_RowsAdded(System::Object^ sender, System::Windows::Forms::DataGridViewRowsAddedEventArgs^ e);
	Void dataGridListFiles_RowsRemoved(System::Object^ sender, System::Windows::Forms::DataGridViewRowsRemovedEventArgs^ e);

	Void btnOpenImage_Click(System::Object^ sender, System::EventArgs^ e);
	Void txtFolderPath_TextChanged(System::Object^ sender, System::EventArgs^ e);
	Void btnFolderPath_Click(System::Object^ sender, System::EventArgs^ e);
	Void txtImageFilter_TextChanged(System::Object^ sender, System::EventArgs^ e);
	Void checkboxFolderRecursive_CheckedChanged(System::Object^ sender, System::EventArgs^ e);
	Void btnFolderLoad_Click(System::Object^ sender, System::EventArgs^ e);
	Void btnClearGridView_Click(System::Object^ sender, System::EventArgs^ e);

	Void btnGenerateSlice_Click(System::Object^ sender, System::EventArgs^ e);

	Void btnStart_Click(System::Object^ sender, System::EventArgs^ e);
	Void btnExit_Click(System::Object^ sender, System::EventArgs^ e);

	Void Inspect();
	void LoadRecipe();
	Void GenerateSlices(String^ path, int start, int end, int step, int blurSize, double maxIten);
	Void UpdateToolStripStatus(String^ msg, int progress);
private:
	String^ mRecipePath = gcnew String("");
	String^ mFolderPath = gcnew String("");
	String^ mFileFilter = gcnew String("");
	String^ mSaveFolder = gcnew String("");
	bool mIsRecursiveSearch = false;
	List<String^>^ mImgList = gcnew List<String^>();
	DataTable^ mDataTable = gcnew DataTable();
	BindingSource^ mGridviewBindingSource = gcnew BindingSource();
	bool mIsUpperPart = true;

	xvt::battery::CTBatteryInspector* mIspLower = new xvt::battery::CTBatteryInspector();
	xvt::battery::CTBatteryInspector* mIspUpper = new xvt::battery::CTBatteryInspector();

private: System::Windows::Forms::Panel^ panel1;
private: System::Windows::Forms::Panel^ panel2;
private: System::Windows::Forms::ComboBox^ cBoxBatteryPart;
private: System::Windows::Forms::ComboBox^ comboBox1;
private: System::Windows::Forms::Label^ label4;
private: System::Windows::Forms::Label^ label5;
private: System::ComponentModel::Container^ components;
private: System::Windows::Forms::Button^ btnOpenImage;
private: System::Windows::Forms::Button^ btnFolderPath;
private: System::Windows::Forms::Button^ btnFolderLoad;
private: System::Windows::Forms::Button^ btnStart;
private: System::Windows::Forms::Button^ btnExit;
private: System::Windows::Forms::Button^ buttonSaveBrowser;
private: System::Windows::Forms::Label^ labelRecipePath;
private: System::Windows::Forms::Label^ labelBatteryPart;
private: System::Windows::Forms::Label^ labelSuffix;
private: System::Windows::Forms::Label^ labelAbout;
private: System::Windows::Forms::Label^ labelFilter;
private: System::Windows::Forms::Label^ labelFormat;
private: System::Windows::Forms::RadioButton^ rButtonFolder;
private: System::Windows::Forms::RadioButton^ rButtonFile;
private: System::Windows::Forms::GroupBox^ gBoxLoadImages;
private: System::Windows::Forms::GroupBox^ gBoxSaveImages;
private: System::Windows::Forms::GroupBox^ gBoxSetting;
private: System::Windows::Forms::GroupBox^ gBoxInspect;
private: System::Windows::Forms::TextBox^ txtFolderPath;
private: System::Windows::Forms::TextBox^ txtImageFilter;
private: System::Windows::Forms::TextBox^ txtSavePath;
private: System::Windows::Forms::TextBox^ tBoxSuffix;
private: System::Windows::Forms::ComboBox^ cBoxFormat;
private: System::Windows::Forms::ProgressBar^ progressBarInspect;
private: System::Windows::Forms::Button^ btnRecipeLoad;
private: System::Windows::Forms::CheckBox^ checkboxFolderRecursive;
private: System::Windows::Forms::TextBox^ txtRecipePath;
private: System::Windows::Forms::Button^ btnRecipePath;
private: System::Windows::Forms::ToolStripContainer^ toolStripContainer1;
private: System::Windows::Forms::StatusStrip^ statusStrip1;
private: System::Windows::Forms::ToolStripStatusLabel^ toolStripStatusLabel1;
private: System::Windows::Forms::Button^ btnClearGridView;
private: System::Windows::Forms::GroupBox^ groupBox1;
private: System::Windows::Forms::ComboBox^ cbxAvgType;
private: System::Windows::Forms::TextBox^ txtVolume;
private: System::Windows::Forms::Label^ label7;
private: System::Windows::Forms::Label^ lblVolume;
private: System::Windows::Forms::NumericUpDown^ numAngleStep;
private: System::Windows::Forms::Label^ label1;
private: System::Windows::Forms::Label^ label3;
private: System::Windows::Forms::Label^ label6;
private: System::Windows::Forms::NumericUpDown^ numAngleEnd;
private: System::Windows::Forms::Button^ btnGenerateSlice;
private: System::Windows::Forms::Label^ label2;
private: System::Windows::Forms::NumericUpDown^ numAngleStart;
private: System::Windows::Forms::NumericUpDown^ numBlurSize;
private: System::Windows::Forms::Label^ label8;
private: System::Windows::Forms::NumericUpDown^ numMax;
private: System::Windows::Forms::DataGridView^ dataGridListFiles;

};
}
