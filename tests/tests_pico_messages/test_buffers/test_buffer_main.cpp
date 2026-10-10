
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <unittest.h>
#include <iobuffer.h>

using namespace ros2_bridge;

int test_simple()
{
    printf("XXXXXXXXXXXHello world from buffer test \n");
    UT_EQUAL_INT(1, 1);
    return 0;
}
int test_make_buffer()
{
    IoBuffer iob{256};
    printf("m_size %ld \n", iob.size());
    printf("m_capacity %ld \n", iob.capacity());
    printf("m_cptr %lx \n", (long)iob.data());
    UT_EQUAL_INT(iob.size(), 0);
    UT_NOT_EQUAL_PTR((void*)iob.data(), NULL);
    return 0;
}
int test_expansion()
{
    std::string extra{"abcedfghijklmnopqrstuvwxyz01923456789"};
    IoBuffer b2{128};
    UT_EQUAL_INT(b2.capacity(), 128)
    for(int i = 0; i < 5; i++) {
        b2.append((void*)extra.c_str(), extra.size());
    }

    printf("b2 m_size %ld \n", b2.size());
    printf("b2 m_capacity %ld \n", b2.capacity());
    printf("b2 m_cptr %lx \n", (long)b2.data());
    UT_EQUAL_INT(5*(extra.size()), b2.size());
    return 0;
}
int test_big_expansion()
{
    std::string sbig{};
    std::string extra{"abcedfghijklmnopqrstuvwxyz01923456789"};
    for(int i = 0; i < 2800; i++) {
        sbig += extra;
    }
    IoBuffer b{};
    b.append(sbig);
    printf("b length %ld \n", b.size());
    printf("b m_size %ld \n", b.size());
    printf("b m_capacity %ld \n", b.capacity());
    printf("b m_cptr %lx \n", (long)b.data());
    printf("sbig.size() %ld \n", (long)sbig.size());
    UT_EQUAL_INT(2800*(extra.size()), b.size());
    return 0;
}
#if 0
// demonstrate clear makes empty without additional allocation or deallocation
int test_cbuffer_clear()
{
    char* s1 = cstr_concat("","");
    char* extra = "abcedfghijklmnopqrstuvwxyz01923456789";
    for(int i = 0; i < 2800; i++) {
        char* s2 = cstr_concat(s1, extra);
        free(s1);
        s1 = s2;
    }
    CbufferRef b2 = Cbuffer_from_cstring(s1);
    void* data1 = Cbuffer_data(b2);
    int sz1 = Cbuffer_size(b2);
    Cbuffer_clear(b2);
    void* data2 = Cbuffer_data(b2);
    int sz2 = Cbuffer_size(b2);
    UT_EQUAL_PTR(data1, data2);
    UT_NOT_EQUAL_INT(sz1, sz2);
    Cbuffer_dispose(&b2);
    UT_EQUAL_PTR(b2, NULL);
    free(s1);
    return 0;
}
#endif

// C++ style move sematics
int test_buffer_move()
{
    std::string s1{};
    std::string extra{"abcedfghijklmnopqrstuvwxyz01923456789"};
    std::string extra2{"1234567890"};
    std::string s2{};
    for(int i = 0; i < 2800; i++) {
        s2 += extra;
    }
    IoBuffer b2{s2};

    IoBuffer b3{std::move(b2)};


    void* d13 = b3.data();
    std::size_t sz13 = b3.size();
    IoBuffer b1{extra2};
    void* d11 = b1.data();
    std::size_t sz11 = b1.size();
    b1 = std::move(b3);
    void* d23 = b3.data();
    std::size_t sz23 = b3.size();
    void* d21 = b1.data();
    std::size_t sz21 = b1.size();

    UT_EQUAL_PTR(d21, d13)
    UT_EQUAL_INT(sz21, sz13)
    return 0;
}
#if 0
int test_chain_make()
{
    BufferChainRef bcr = BufferChain_new();
    char* s1 = cstr_concat("","");
    char* extra = "abcedfghijklmnopqrstuvwxyz01923456789";
    for(int i = 0; i < 2800; i++) {
        BufferChain_append(bcr, (void*)extra, strlen(extra));
    }
    int x = BufferChain_size(bcr);
    UT_EQUAL_INT(BufferChain_size(bcr), 2800*strlen(extra))
    BufferChain_dispose(&bcr);
    UT_EQUAL_PTR(bcr, NULL);

    return 0;
}
int test_chain_compact() // and eq_cstr
{
    BufferChainRef bcr = BufferChain_new();
    char* s1 = cstr_concat("","");
    char* s2;
    char* extra = "abcedfghijklmnopqrstuvwxyz01923456789";
    for(int i = 0; i < 2800; i++) {
        BufferChain_append(bcr, (void*)extra, strlen(extra));
        s2 = cstr_concat(s1, extra);
        s1 = s2;
    }
    UT_EQUAL_INT(BufferChain_size(bcr), 2800*strlen(extra))
    IOBufferRef iob = BufferChain_compact(bcr);
//    int x = strlen((char*)IOBuffer_data(iob));
//    UT_EQUAL_INT(x, IOBuffer_data_len(iob));

    int y = strncmp(s1, (char*)IOBuffer_data(iob), strlen(s1));

    bool ok = BufferChain_eq_cstr(bcr, s1);
    UT_EQUAL_INT(ok, 1);
    s1[3] = 'X';
    bool ok2 = BufferChain_eq_cstr(bcr, s1);
    UT_EQUAL_INT(ok2, 0);
    BufferChain_dispose(&bcr);
    UT_EQUAL_PTR(bcr, NULL);
    IOBuffer_dispose(&iob);
    UT_EQUAL_PTR(iob, NULL);
    return 0;
}
BufferChainRef make_chain_1()
{
    char* str[5] = {
            (char*)"abcdefg",
            (char*)"hijklmno",
            (char*)"pqrstuvw",
            (char*)"xyz",
            NULL
    };
    BufferChainRef bc = BufferChain_new();
    for(int i = 0; i < 4; i++) {
        IOBufferRef iob = IOBuffer_from_cstring(str[i]);
        BufferChain_add_back(bc, iob);
    }
    return bc;
}
BufferChainRef make_chain_2()
{
    char* str[5] = {
            (char*)"ABCDEFGH",
            (char*)"IJKLMNOPQ",
            (char*)"RSTUVWXYZ1234",
            NULL
    };
    BufferChainRef bc = BufferChain_new();
    for(int i = 0; i < 3; i++) {
        IOBufferRef iob = IOBuffer_from_cstring(str[i]);
        BufferChain_add_back(bc, iob);
    }
    return bc;
}

int test_chain_front()
{
    BufferChainRef bc = make_chain_1();
    while(BufferChain_size(bc) != 0) {
        int size_1 = BufferChain_size(bc);
        IOBufferRef iob = BufferChain_pop_front(bc);
        int iob_size1 = IOBuffer_data_len(iob);
        int size_2 = BufferChain_size(bc);
        UT_EQUAL_INT(size_2, size_1 - iob_size1);
    }
    return 0;
}
int test_chain_steal()
{
    BufferChainRef bc1 = make_chain_1();
    BufferChainRef bc2 = make_chain_2();
    int size1 = BufferChain_size(bc1);
    int size2 = BufferChain_size(bc2);
    BufferChain_steal_bufferchain(bc1, bc2);
    UT_EQUAL_INT(BufferChain_size(bc1), size1 + size2);
    UT_EQUAL_INT(BufferChain_size(bc2), 0);                // notice this - this is the different between append and steal
    IOBufferRef compacted = BufferChain_compact(bc1);
    const char* s = IOBuffer_cstr(compacted);
    IOBuffer_dispose(&compacted);
    return 0;
}
int test_chain_append()
{
    BufferChainRef bc1 = make_chain_1();
    BufferChainRef bc2 = make_chain_2();
    int size1 = BufferChain_size(bc1);
    int size2 = BufferChain_size(bc2);
    BufferChain_append_bufferchain(bc1, bc2);
    UT_EQUAL_INT(BufferChain_size(bc1), size1 + size2);
    UT_EQUAL_INT(BufferChain_size(bc2), size2);                // notice this - this is the different between append and steal
    IOBufferRef compacted = BufferChain_compact(bc1);
    const char* s = IOBuffer_cstr(compacted);
    IOBuffer_dispose(&compacted);
    return 0;
}

IOBufferRef make_iobuffer()
{
    IOBufferRef iob = IOBuffer_from_cstring("abcdefghijklmnopqrstuvwxyz");
    IOBuffer_consume(iob, 10);
    return iob;
}
int test_iobuffer_dup()
{
    IOBufferRef iob = make_iobuffer();
    IOBufferRef iob_dup = IOBuffer_dup(iob);
    UT_EQUAL_INT(IOBuffer_data_len(iob), IOBuffer_data_len(iob_dup));
    UT_NOT_EQUAL_PTR(IOBuffer_data(iob), IOBuffer_data(iob_dup));

    UT_EQUAL_INT(IOBuffer_space_len(iob), IOBuffer_space_len(iob_dup));
    UT_NOT_EQUAL_PTR(IOBuffer_space(iob), IOBuffer_space(iob_dup));
    const char* s = IOBuffer_cstr(iob);
    const char* s_dup = IOBuffer_cstr(iob_dup);
    UT_EQUAL_INT(strcmp(s, s_dup), 0);
    return 0;
}
int test_iobuffer_make()
{
    IOBufferRef ioref = IOBuffer_new();
    void* x = IOBuffer_space(ioref);
    int l = IOBuffer_space_len(ioref);
    char* sconst = "A0123456789P";
    size_t y = strlen(sconst);
    memcpy(x, sconst, y+1);
    IOBuffer_commit(ioref, y+1);
    void* data = IOBuffer_data(ioref);
    int data_length = IOBuffer_data_len(ioref);
    IOBuffer_consume(ioref, 1);
    void* data_1 = IOBuffer_data(ioref);
    int data_length_1 = IOBuffer_data_len(ioref);
    UT_EQUAL_PTR((data+1), IOBuffer_data(ioref));
    UT_EQUAL_INT((data_length - 1), IOBuffer_data_len(ioref));
    UT_EQUAL_INT(strcmp("0123456789P", (char*)IOBuffer_data(ioref)), 0);
    IOBuffer_dispose(&ioref);
    return 0;
}
#endif

int test_iobuffer_consume_commit_01()
{
    IoBuffer iob{1024};
    std::size_t len1 = iob.space_len();
    std::string sconst{"A0123456789P"};
    std::size_t y = sconst.size();
    std::string total_string{};

    for(int j; j < 10; j++) {
        total_string += sconst;
        // simulate a read operation into the 'space'
        void* p1 = iob.space_ptr();
        memcpy(p1, sconst.c_str(), y); // this time dont copy the zero terminator
        // now update the iob to account for the new data
        iob.commit(y);

        std::string test_string{iob.to_string()};

        bool b1 = (test_string == iob.to_string());
        bool b2 = (total_string == test_string);
    }
    std:size_t count = iob.size();
    std::size_t i = 0;
    while(iob.size() > 0) {
        std::string string_before{iob.to_string()};
        iob.consume(1);
        std::string string_after{iob.to_string()};
        bool b3 = (string_before.substr(1) == string_after);
        UT_TRUE(b3)
        count -= 1;
        i += 1;
        UT_EQUAL_INT(count, iob.size())
    }
    UT_TRUE((iob.to_string() == std::string("")));
    UT_EQUAL_INT(count, 0)
    return 0;
}
#if 0
int test_iobuffer_commit_consume_extra()
{
    char* s1 = "0987654321";
    char* s2 = "abcdefghijklmno";
    char* s3;
    int l = asprintf(&s3, "%s%s", s1,s2);

    IOBufferRef iob = IOBuffer_new_with_capacity(strlen(s3));

    // space pointer becomes data pointer after adding data
    void* p_start = IOBuffer_space(iob);
    IOBuffer_data_add(iob, s1, strlen(s1));
    void* tmp1 = IOBuffer_data(iob);
    UT_EQUAL_PTR(p_start, IOBuffer_data(iob));

    // after adding data IOBuffer_data_len() should give len of string added
    int len_1 = IOBuffer_data_len(iob);
    UT_EQUAL_INT(len_1, strlen(s1));

    // space pointer after one add shoud be start + len of string added
    void* p1 = (IOBuffer_data(iob) + len_1);
    void* space_after_1 = IOBuffer_space(iob);
    UT_EQUAL_PTR(p1, space_after_1);

    IOBuffer_data_add(iob, s2, strlen(s2));
    // after adding a second string designed to fill capacity space_len2 should be zero
    void* space2 = IOBuffer_space(iob);
    int space_len2 = IOBuffer_space_len(iob);
    UT_EQUAL_INT(space_len2, 0);
    // space pointer - not sure
    void* pc = (void*)(p_start + strlen(s1) + strlen(s2));
    UT_EQUAL_PTR(space2, pc);

    // data length should be sum of lengths of added strings
    int len_2 = IOBuffer_data_len(iob);
    UT_EQUAL_INT(len_2, strlen(s1) + strlen(s2));

    const char* str = IOBuffer_cstr(iob);

    UT_EQUAL_INT(0, strcmp(s3, str))
    // consume s1 should only leave s2
    IOBuffer_consume(iob, strlen(s1));
    UT_EQUAL_INT(strlen(s2), IOBuffer_data_len(iob));
    UT_EQUAL_INT(strcmp(IOBuffer_cstr(iob), s2), 0);

    // consume s2 should leave nothing
    IOBuffer_consume(iob, strlen(s2));
    UT_EQUAL_INT(IOBuffer_data_len(iob), 0);
    UT_EQUAL_PTR(IOBuffer_space(iob), p_start);

    // final test when buffer is empty data pointer should be p_start
    UT_EQUAL_PTR(p_start, IOBuffer_data(iob));
    printf("this is the end");
    return 0;
}
#endif
int main()
{
    UT_ADD(test_simple);
    UT_ADD(test_make_buffer);
    UT_ADD(test_expansion);
    UT_ADD(test_big_expansion);
    UT_ADD(test_buffer_move);
    UT_ADD(test_iobuffer_consume_commit_01);
//    UT_ADD(test_chain_front);
//    UT_ADD(test_iobuffer_commit_consume_extra);
//    UT_ADD(test_make_buffer);
//    UT_ADD(test_expansion);
//    UT_ADD(test_big_expansion);
//    UT_ADD(test_cbuffer_clear);
//    UT_ADD(test_cbuffer_move);
//    UT_ADD(test_chain_make);
//    UT_ADD(test_chain_compact);
//    UT_ADD(test_iobuffer_make);
//    UT_ADD(test_iobuffer_make2);
    int rc = UT_RUN();
    return rc;
}