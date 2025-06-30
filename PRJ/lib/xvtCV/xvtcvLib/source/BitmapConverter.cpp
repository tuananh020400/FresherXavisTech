#include "xvtCV/BitmapConverter.h"
#ifdef __CLR__
#include <opencv2/imgproc/imgproc.hpp>
namespace xvt {
cv::Mat BitmapConverter::ToMat(Bitmap^ src)
{
    if (src == nullptr)
        throw gcnew System::ArgumentNullException(src->ToString());

    int w = src->Width;
    int h = src->Height;
    int channels;
    switch (src->PixelFormat)
    {
    case PixelFormat::Format24bppRgb:
    case PixelFormat::Format32bppRgb:
        channels = 3; break;
    case PixelFormat::Format32bppArgb:
    case PixelFormat::Format32bppPArgb:
        channels = 4; break;
    case PixelFormat::Format8bppIndexed:
    case PixelFormat::Format1bppIndexed:
        channels = 1; break;
    default:
        throw gcnew System::NotImplementedException();
    }

    cv::Mat dst = cv::Mat(h, w, CV_8UC(channels));
    ToMat(src, dst);
    return dst;
}

void Ch1(cv::Mat dst, int height, int srcStep, uint dstStep, IntPtr srcData, System::Byte* palette)
{
    if (dstStep == srcStep && !dst.isSubmatrix() && dst.isContinuous())
    {
        // Read Bitmap pixel data to managed array
        long length = (int64_t)dst.dataend - (int64_t)dst.data;
        if (length > INT_MAX)
            throw gcnew NotSupportedException("Too big dst Mat");
        System::Byte* buffer = new System::Byte[length];

        memcpy(dst.data, srcData.ToPointer(), srcStep);
    }
    else
    {
        // Copy line bytes from src to dst for each line
        System::Byte* sp = (System::Byte*)srcData.ToPointer();
        System::Byte* dp = (System::Byte*)dst.data;
        auto buffer = new System::Byte[srcStep];
        for (int y = 0; y < height; y++)
        {
            memcpy(dp, sp,dstStep);
            sp += srcStep;
            dp += dstStep;
        }
    }
}

void BitmapConverter::ToMat(Bitmap^ src, cv::Mat dst)
{
    if (src == nullptr)
        throw gcnew ArgumentNullException("src is nullptr.");
    if (dst.empty())
        throw gcnew ArgumentNullException("The specified dst is empty.");
    if (dst.depth() !=CV_8U)
        throw gcnew NotSupportedException("Mat depth != CV_8U");
    if (dst.dims != 2)
        throw gcnew NotSupportedException("Mat dims != 2");
    if (src->Width != dst.cols || src->Height != dst.rows)
        throw gcnew ArgumentException("src.Size != dst.Size");

    int w = src->Width;
    int h = src->Height;
    Rectangle rect = Rectangle(0, 0, w, h);
    BitmapData^ bd = nullptr;
    try
    {
        bd = src->LockBits(rect, ImageLockMode::ReadOnly, src->PixelFormat);

        switch (src->PixelFormat)
        {
        case PixelFormat::Format1bppIndexed:
        {
            if (dst.channels() != 1)
                throw gcnew ArgumentException("Invalid nChannels");
            if (dst.isSubmatrix())
                throw gcnew NotImplementedException("submatrix not supported");
            if (bd == nullptr)
                throw gcnew NotSupportedException("BitmapData == null (Format1bppIndexed)");

            System::Byte* srcPtr = reinterpret_cast<System::Byte*>(bd->Scan0.ToPointer());
            System::Byte* dstPtr = dst.data;
            int srcStep = bd->Stride;
            uint dstStep = (uint)dst.step;
            int x = 0;

            for (int y = 0; y < h; y++)
            {
                for (int bytePos = 0; bytePos < srcStep; bytePos++)
                {
                    if (x < w)
                    {
                        System::Byte b = srcPtr[bytePos];
                        for (int i = 0; i < 8; i++)
                        {
                            if (x >= w)
                            {
                                break;
                            }
                            dstPtr[dstStep * y + x] = ((b & 0x80) == 0x80) ? (System::Byte)255 : (System::Byte)0;
                            b <<= 1;
                            x++;
                        }
                    }
                }
                x = 0;
                srcPtr += srcStep;
            }
            break;
        }
        case PixelFormat::Format8bppIndexed:
        {
            int srcStep = bd->Stride;
            uint dstStep = (uint)dst.step;

            int channels = dst.channels();
            if (channels == 1)
            {
                System::Byte palette[256];
                int paletteLength = Math::Min(256, src->Palette->Entries->Length);
                for (int i = 0; i < paletteLength; i++)
                {
                    // TODO src.Palette.Flags & 2 == 2
                    // https://docs.microsoft.com/ja-jp/dotnet/api/system.drawing.imaging.colorpalette.flags?view=netframework-4.8
                    palette[i] = src->Palette->Entries[i].R;
                }
                Ch1(dst, h, srcStep, dstStep, bd->Scan0, palette);
            }
            else if (channels == 3)
            {
                // Palette
                auto paletteR = new System::Byte[256];
                auto paletteG = new System::Byte[256];
                auto paletteB = new System::Byte[256];
                auto paletteLength = Math::Min(256, src->Palette->Entries->Length);
                for (int i = 0; i < paletteLength; i++)
                {
                    auto c = src->Palette->Entries[i];
                    paletteR[i] = c.R;
                    paletteG[i] = c.G;
                    paletteB[i] = c.B;
                }

                cv::Mat dstR = cv::Mat(h, w, CV_8UC1);
                cv::Mat dstG = cv::Mat(h, w, CV_8UC1);
                cv::Mat dstB = cv::Mat(h, w, CV_8UC1);

                Ch1(dstR, h, srcStep, (uint)dstR.step, bd->Scan0, paletteR);
                Ch1(dstG, h, srcStep, (uint)dstG.step, bd->Scan0, paletteG);
                Ch1(dstB, h, srcStep, (uint)dstB.step, bd->Scan0, paletteB);
                std::vector<cv::Mat> channels;
                channels.push_back(dstB);
                channels.push_back(dstG);
                channels.push_back(dstR);
                cv::merge(channels, dst);
            }
            else
            {
                throw gcnew ArgumentException("Invalid channels of dst Mat ({channels})");
            }
            break;
        }
        case PixelFormat::Format24bppRgb:
        {
            if (dst.channels() != 3)
                throw gcnew ArgumentException("Invalid nChannels");
            if (dst.depth() != CV_8U && dst.depth() != CV_8S)
                throw gcnew ArgumentException("Invalid depth of dst Mat");

            int srcStep = bd->Stride;
            long dstStep = dst.step;
            if (dstStep == srcStep && !dst.isSubmatrix() && dst.isContinuous())
            {
                IntPtr dstData = IntPtr(dst.data);
                long bytesToCopy = (int64_t)dst.dataend - dstData.ToInt64();
                memcpy(bd->Scan0.ToPointer(), dstData.ToPointer(), bytesToCopy);
            }
            else
            {
                // Copy line bytes from src to dst for each line
                IntPtr sp = bd->Scan0;
                IntPtr dp = IntPtr(dst.data);
                for (int y = 0; y < h; y++)
                {
                    memcpy(sp.ToPointer(), dp.ToPointer(), dstStep);
                    sp += srcStep;
                    dp += dstStep;
                }
            }
            break;
        }
        case PixelFormat::Format32bppRgb:
        case PixelFormat::Format32bppArgb:
        case PixelFormat::Format32bppPArgb:
        {
            int srcStep = bd->Stride;
            long dstStep = dst.step;

            switch (dst.channels())
            {
            case 4:
            {
                if (!dst.isSubmatrix() && dst.isContinuous())
                {
                    IntPtr dstData = IntPtr(dst.data);
                    long bytesToCopy = (int64_t)dst.dataend - dstData.ToInt64();
                    memcpy(bd->Scan0.ToPointer(), dstData.ToPointer(), bytesToCopy);
                }
                else
                {
                    IntPtr sp = bd->Scan0;
                    IntPtr dp = IntPtr(dst.data);
                    for (int y = 0; y < h; y++)
                    {
                        memcpy(sp.ToPointer(), dp.ToPointer(), dstStep);
                        sp += srcStep;
                        dp += dstStep;
                    }
                }

                break;
            }
            case 3:
            {
                System::Byte* srcPtr = (System::Byte*)bd->Scan0.ToPointer();
                System::Byte* dstPtr = (System::Byte*)dst.data;
                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        dstPtr[y * dstStep + x * 3 + 0] = srcPtr[y * srcStep + x * 4 + 0];
                        dstPtr[y * dstStep + x * 3 + 1] = srcPtr[y * srcStep + x * 4 + 1];
                        dstPtr[y * dstStep + x * 3 + 2] = srcPtr[y * srcStep + x * 4 + 2];
                    }
                }

                break;
            }
            default:
                throw gcnew ArgumentException("Invalid nChannels");
            }
            break;
        }
        }
    }
    finally
    {
        if (bd != nullptr)
            src->UnlockBits(bd);
    }
}

Bitmap^ BitmapConverter::ToBitmap(cv::Mat src)
{
    if (src.empty())
    {
        throw gcnew ArgumentNullException("this src is empty.");
    }

    PixelFormat pf;
    switch (src.channels())
    {
    case 1:
        pf = PixelFormat::Format8bppIndexed; break;
    case 3:
        pf = PixelFormat::Format24bppRgb; break;
    case 4:
        pf = PixelFormat::Format32bppArgb; break;
    default:
        throw gcnew ArgumentException("Number of channels must be 1, 3 or 4.");
    }
    return ToBitmap(src, pf);
}

Bitmap^ BitmapConverter::ToBitmap(cv::Mat src, PixelFormat pf)
{
    if (src.empty())
        throw gcnew ArgumentNullException("this src is empty.");

    Bitmap^ bitmap = gcnew Bitmap(src.cols, src.rows, pf);
    ToBitmap(src, bitmap);
    return bitmap;
}

void BitmapConverter::ToBitmap(cv::Mat src, Bitmap^ dst)
{
    if (src.empty())
        throw gcnew ArgumentNullException("src is nullptr.");
    if (dst == nullptr)
        throw gcnew ArgumentNullException("The specified dst is empty.");
    if (src.depth() != CV_8U)
        throw gcnew ArgumentException("Depth of the image must be CV_8U");
    //if (src.IsSubmatrix())
    //    throw new ArgumentException("Submatrix is not supported");
    if (src.cols != dst->Width || src.rows != dst->Height)
        throw gcnew ArgumentException("");

    PixelFormat pf = dst->PixelFormat;

    if (pf == PixelFormat::Format8bppIndexed)
    {
        ColorPalette^ plt = dst->Palette;
        for (int x = 0; x < 256; x++)
        {
            plt->Entries[x] = Color::FromArgb(x, x, x);
        }
        dst->Palette = plt;
    }

    int w = src.cols;
    int h = src.rows;
    Rectangle rect = Rectangle(0, 0, w, h);
    BitmapData^ bd = nullptr;

    bool submat = src.isSubmatrix();
    bool continuous = src.isContinuous();

    try
    {
        bd = dst->LockBits(rect, ImageLockMode::WriteOnly, pf);

        IntPtr srcData = IntPtr(src.data);
        System::Byte* pSrc = (System::Byte*)(srcData.ToPointer());
        System::Byte* pDst = (System::Byte*)(bd->Scan0.ToPointer());
        int ch = src.channels();
        int srcStep = (int)src.step;
        int dstStep = ((src.cols * ch) + 3) / 4 * 4;
        int stride = bd->Stride;

        switch (pf)
        {
        case PixelFormat::Format1bppIndexed:
        {
            if (submat)
                throw gcnew NotImplementedException("submatrix not supported");

            //int offset = stride - (w / 8);
            int x = 0;
            System::Byte b = 0;
            for (int y = 0; y < h; y++)
            {
                for (int bytePos = 0; bytePos < stride; bytePos++)
                {
                    if (x < w)
                    {
                        for (int i = 0; i < 8; i++)
                        {
                            auto mask = (System::Byte)(0x80 >> i);
                            if (x < w && pSrc[srcStep * y + x] == 0)
                                b &= (System::Byte)(mask ^ 0xff);
                            else
                                b |= mask;

                            x++;
                        }
                        pDst[bytePos] = b;
                    }
                }
                x = 0;
                pDst += stride;
            }
            break;
        }
        case PixelFormat::Format8bppIndexed:
        case PixelFormat::Format24bppRgb:
        case PixelFormat::Format32bppArgb:
        {
            if (srcStep == dstStep && !submat && continuous)
            {
                long bytesToCopy = (int64_t)src.dataend - (int64_t)src.data;
                memcpy(reinterpret_cast<unsigned char*>(pDst), reinterpret_cast<unsigned char*>(pSrc), bytesToCopy);
            }
            else
            {
                for (int y = 0; y < h; y++)
                {
                    long offsetSrc = (y * srcStep);
                    long offsetDst = (y * dstStep);
                    long bytesToCopy = w * ch;
                    memcpy(pDst + offsetDst, pSrc + offsetSrc, bytesToCopy);
                }
            }
            break;
        }
        default:
            throw gcnew NotImplementedException();
        }
    }
    finally
    {
        if (bd != nullptr)
            dst->UnlockBits(bd);
    }
}
}
#endif
